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
    // Dark outline for readability
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
    cout << "=== Stereo Viewer ===" << endl;
    cout << "Resolution: " << width << "x" << height << " @ " << fps << " fps" << endl;
    cout << "Press ESC to quit, SPACE to save snapshot" << endl;

    /* -------------------------------------------------------------
       Open cameras with the working GStreamer pipeline.
       videoconvert forces a CPU-memory copy so appsink can read it.
       ------------------------------------------------------------- */
    cout << "Opening camera 0 (right)..." << endl;
    VideoCapture cam0(buildPipeline(1, width, height, fps), CAP_GSTREAMER);
    cout << "Opening camera 1 (left)..." << endl;
    usleep(500000);  // 500 ms delay: avoid nvargus-daemon race
    VideoCapture cam1(buildPipeline(0, width, height, fps), CAP_GSTREAMER);

    if (!cam0.isOpened()) {
        cerr << "ERROR: Camera 0 (sensor-id=0) failed to open!" << endl;
        return -1;
    }
    if (!cam1.isOpened()) {
        cerr << "ERROR: Camera 1 (sensor-id=1) failed to open!" << endl;
        return -1;
    }
    cout << "Both cameras online." << endl;

    /* -------------------------------------------------------------
       StereoBM setup
       ndisparities must be a multiple of 16.
       SADWindowSize must be odd and >= 5.
       ------------------------------------------------------------- */
    int ndisparities  = 16 * 5;   // 80 px disparity search range
    int SADWindowSize = 21;       // matched block size
    Ptr<StereoBM> bm  = StereoBM::create(ndisparities, SADWindowSize);

    Mat frame0, frame1;   // BGR from GStreamer
    Mat gray0, gray1;     // grayscale for StereoBM
    Mat disp16S;          // raw 16-bit fixed-point disparity
    Mat disp8U;           // normalized 8-bit for display
    Mat dispColor;        // color-mapped "depth" image

    namedWindow("Right  (Cam0)", WINDOW_AUTOSIZE);
    namedWindow("Left   (Cam1)", WINDOW_AUTOSIZE);
    namedWindow("Depth  (Disparity)", WINDOW_AUTOSIZE);

    auto lastTime   = chrono::high_resolution_clock::now();
    int frameCount  = 0;
    double currFps  = 0.0;

    while (g_keepRunning) {
        cam0 >> frame0;
        cam1 >> frame1;

        if (frame0.empty() || frame1.empty()) {
            cerr << "Warning: empty frame, skipping..." << endl;
            continue;
        }

        /* ---- Grayscale for stereo matching ---- */
        cvtColor(frame0, gray0, COLOR_BGR2GRAY);
        cvtColor(frame1, gray1, COLOR_BGR2GRAY);

        /* ---- Compute disparity ----
           StereoBM::compute(leftImage, rightImage, disparity)
           disp16S values are fixed-point: real_disparity = value / 16.0
        */
        bm->compute(gray0, gray1, disp16S);

        /* ---- Normalize to 8-bit for human-friendly viewing ---- */
        double minVal, maxVal;
        minMaxLoc(disp16S, &minVal, &maxVal);

        // Avoid divide-by-zero
        if (maxVal <= minVal) maxVal = minVal + 1;

        disp16S.convertTo(disp8U, CV_8UC1,
                          255.0 / (maxVal - minVal),
                          -minVal * 255.0 / (maxVal - minVal));

        // COLORMAP_JET: red = close, blue = far  (very intuitive)
        applyColorMap(disp8U, dispColor, COLORMAP_JET);

        /* ---- FPS counter ---- */
        ++frameCount;
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double>(now - lastTime).count();
        if (elapsed >= 1.0) {
            currFps   = frameCount / elapsed;
            frameCount = 0;
            lastTime   = now;
        }

        string fpsTxt = "FPS: " + to_string(int(currFps));
        drawText(frame0, fpsTxt + "  Right",  Point(10, 30), Scalar(0, 255, 0));
        drawText(frame1, fpsTxt + "  Left",   Point(10, 30), Scalar(0, 255, 0));
        drawText(dispColor, fpsTxt + "  Depth", Point(10, 30), Scalar(255, 255, 255));

        // Show true disparity range (divide by 16 because of fixed-point format)
        string dRange = "Disp: " + to_string(int(minVal / 16.0))
                        + " - " + to_string(int(maxVal / 16.0)) + " px";
        putText(dispColor, dRange, Point(10, 55),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

        imshow("Left", frame0);
        imshow("Right", frame1);
        imshow("Depth  (Disparity)", dispColor);

        char key = (char)waitKey(1);
        if (key == 27) break;                     // ESC
        if (key == ' ') {                         // SPACE = snapshot
            imwrite("snapshot_right.png",  frame0);
            imwrite("snapshot_left.png",   frame1);
            imwrite("snapshot_depth.png",  dispColor);
            cout << "Snapshots saved!" << endl;
        }
    }

    cam0.release();
    cam1.release();
    destroyAllWindows();
    cout << "Releasing cameras..." << endl;
    cam0.release();
    cam1.release();
    destroyAllWindows();
    return 0;
}
