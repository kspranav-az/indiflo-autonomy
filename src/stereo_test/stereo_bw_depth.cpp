#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <chrono>
#include <csignal>
#include <atomic>

using namespace cv;
using namespace std;

static std::atomic<bool> g_keepRunning{true};
void onSignal(int) { g_keepRunning = false; }

string buildPipeline(int sensorId, int width, int height, int fps)
{
    stringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensorId
       << " ! video/x-raw(memory:NVMM), width=" << width
       << ", height=" << height << ", format=NV12, framerate=" << fps
       << "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=" << width
       << ", height=" << height
       << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true";
    return ss.str();
}

void drawText(Mat& img, const string& text, const Point& pos,
              const Scalar& color = Scalar(0, 255, 0))
{
    putText(img, text, pos + Point(1, 1), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
    putText(img, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

int main(int argc, char** argv)
{
    int width  = 640;
    int height = 480;
    int fps    = 30;

    if (argc > 1) width  = atoi(argv[1]);
    if (argc > 2) height = atoi(argv[2]);
    if (argc > 3) fps    = atoi(argv[3]);

    struct sigaction sa;
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    cout << "=== Stereo B&W Depth Viewer ===" << endl;
    cout << "Res: " << width << "x" << height << " @ " << fps << "fps" << endl;
    cout << "White = CLOSE  |  Black = FAR" << endl;
    cout << "ESC = quit  |  SPACE = save snapshot" << endl;

    VideoCapture cam0(buildPipeline(1, width, height, fps), CAP_GSTREAMER);
    usleep(500000);  // 500 ms delay: avoid nvargus-daemon race
    VideoCapture cam1(buildPipeline(0, width, height, fps), CAP_GSTREAMER);

    if (!cam0.isOpened()) { cerr << "Cam0 failed\n"; return -1; }
    if (!cam1.isOpened()) { cerr << "Cam1 failed\n"; return -1; }

    /* StereoBM: ndisparities multiple of 16, blockSize odd >= 5 */
    int ndisparities  = 16 * 5;   // 80 px search range
    int SADWindowSize = 21;
    Ptr<StereoBM> bm  = StereoBM::create(ndisparities, SADWindowSize);

    namedWindow("Left", WINDOW_AUTOSIZE);
    namedWindow("Right", WINDOW_AUTOSIZE);
    namedWindow("Depth B&W", WINDOW_AUTOSIZE);

    auto lastTime  = chrono::high_resolution_clock::now();
    int frameCount = 0;
    double currFps = 0.0;

    Mat frame0, frame1;
    Mat gray0, gray1;
    Mat disp16S;      // StereoBM raw output (16-bit fixed point)
    Mat dispFloat;    // true disparity = value / 16.0
    Mat validMask;    // mask of valid disparities
    Mat depthBW;      // final 8-bit grayscale depth image

    while (g_keepRunning) {
        cam0 >> frame0;
        cam1 >> frame1;
        if (frame0.empty() || frame1.empty()) continue;

        /* 1. Grayscale for stereo matching */
        cvtColor(frame0, gray0, COLOR_BGR2GRAY);
        cvtColor(frame1, gray1, COLOR_BGR2GRAY);

        /* 2. Compute disparity
              compute(left, right, disparity)
              disp16S values are fixed-point: real disparity = value / 16.0   */
        bm->compute(gray0, gray1, disp16S);

        /* 3. Convert to true disparity (float) */
        disp16S.convertTo(dispFloat, CV_32F, 1.0 / 16.0);

        /* 4. Mask invalid pixels (StereoBM marks bad matches with very small
              or negative values). Only keep pixels with disparity > 1 px. */
        validMask = dispFloat > 1.0f;

        /* 5. Find min/max among VALID disparities */
        double minDisp = 0.0, maxDisp = 1.0;
        minMaxLoc(dispFloat, &minDisp, &maxDisp, NULL, NULL, validMask);
        if (maxDisp <= minDisp) maxDisp = minDisp + 1.0;

        /* 6. Normalize to 0-255 (black & white)
           High disparity (close)  -> 255 (white)
           Low disparity  (far)    -> 0   (black)   */
        Mat scaled = (dispFloat - minDisp) * (255.0 / (maxDisp - minDisp));
        scaled.convertTo(depthBW, CV_8UC1);

        // Zero-out invalid pixels so they stay pure black
        depthBW.setTo(0, ~validMask);

        /* 7. Optional median blur to kill StereoBM speckle noise */
        medianBlur(depthBW, depthBW, 3);

        /* 8. FPS counter */
        ++frameCount;
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double>(now - lastTime).count();
        if (elapsed >= 1.0) {
            currFps    = frameCount / elapsed;
            frameCount = 0;
            lastTime   = now;
        }

        string fpsTxt = "FPS: " + to_string(int(currFps));
        drawText(frame0, fpsTxt + " Right", Point(10, 30));
        drawText(frame1, fpsTxt + " Left",  Point(10, 30));
        drawText(depthBW, fpsTxt + " Depth", Point(10, 30), Scalar(255));

        string info = "W=Close B=Far | Disp: "
                      + to_string(int(minDisp)) + "-" + to_string(int(maxDisp)) + "px";
        putText(depthBW, info, Point(10, 55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180), 1);

        imshow("Left", frame0);
        imshow("Right", frame1);
        imshow("Depth B&W", depthBW);

        char key = (char)waitKey(1);
        if (key == 27) break;          // ESC
        if (key == ' ') {              // SPACE = save snapshots
            imwrite("bw_left.png",  frame1);
            imwrite("bw_right.png", frame0);
            imwrite("bw_depth.png", depthBW);
            cout << "Snapshots saved.\n";
        }
    }
    cout << "Releasing cameras..." << endl;
    cam0.release();
    cam1.release();
    destroyAllWindows();
    return 0;
}
