#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <chrono>
#include <cmath>
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
    putText(img, text, pos + Point(1, 1), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 0, 0), 2);
    putText(img, text, pos, FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
}

int main(int argc, char** argv)
{
    int width  = 640;
    int height = 480;
    int fps    = 30;

    if (argc > 1) width  = atoi(argv[1]);
    if (argc > 2) height = atoi(argv[2]);
    if (argc > 3) fps    = atoi(argv[3]);

    /* =============================================================
       CAMERA PARAMETERS  (Sony IMX219 stereo pair)
       Baseline = 60 mm, HFOV = 73 degrees
       ============================================================= */
    const double BASELINE_M = 0.060;
    const double HFOV_DEG   = 73.0;
    const double fx_pixels  = (width / 2.0) / tan(HFOV_DEG * CV_PI / 360.0);
    const double depth_scale = fx_pixels * BASELINE_M;

    const double HARD_MIN_M = 0.20;   // absolute clip: closer than 20 cm
    const double HARD_MAX_M = 10.0;   // absolute clip: farther than 10 m

    struct sigaction sa;
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    cout << "=== Stereo Metric Depth Viewer ===" << endl;
    cout << "Res: " << width << "x" << height << endl;
    cout << "fx (px): " << fixed << setprecision(1) << fx_pixels << endl;
    cout << "Depth = " << depth_scale << " / disparity  (meters)" << endl;
    cout << "Controls: ESC=quit | SPACE=save | L=toggle log/linear" << endl;

    VideoCapture cam0(buildPipeline(1, width, height, fps), CAP_GSTREAMER);
    usleep(500000);  // 500 ms delay: avoid nvargus-daemon race
    VideoCapture cam1(buildPipeline(0, width, height, fps), CAP_GSTREAMER);
    if (!cam0.isOpened()) { cerr << "Cam0 failed\n"; return -1; }
    if (!cam1.isOpened()) { cerr << "Cam1 failed\n"; return -1; }

    /* -------------------------------------------------------------
       StereoBM with tuned parameters (much cleaner than defaults)
       ------------------------------------------------------------- */
    int numDisp    = 16 * 5;      // must be multiple of 16
    int blockSize  = 11;          // odd, 5..21
    Ptr<StereoBM> bm = StereoBM::create(numDisp, blockSize);
    bm->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
    bm->setPreFilterSize(9);
    bm->setPreFilterCap(31);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);

    namedWindow("Left", WINDOW_AUTOSIZE);
    namedWindow("Right", WINDOW_AUTOSIZE);
    namedWindow("Metric Depth (m)", WINDOW_AUTOSIZE);

    auto lastTime  = chrono::high_resolution_clock::now();
    int frameCount = 0;
    double currFps = 0.0;
    bool useLog    = true;   // L key toggles

    Mat frame0, frame1, gray0, gray1;
    Mat disp16S, dispFloat;
    Mat depthMeters(height, width, CV_32F);
    Mat depthDisplay(height, width, CV_8UC1);

    while (g_keepRunning) {
        cam0 >> frame0;
        cam1 >> frame1;
        if (frame0.empty() || frame1.empty()) continue;

        cvtColor(frame0, gray0, COLOR_BGR2GRAY);
        cvtColor(frame1, gray1, COLOR_BGR2GRAY);

        /* ---- Disparity ---- */
        bm->compute(gray0, gray1, disp16S);
        disp16S.convertTo(dispFloat, CV_32F, 1.0 / 16.0);

        // Valid mask: disparity > 1.0 and not inf/nan
        Mat validMask = (dispFloat > 1.0f) & (dispFloat < numDisp);

        /* ---- Metric depth ---- */
        Mat depthTemp = depth_scale / dispFloat;

        // Clip absolute limits first
        Mat wayTooClose = depthTemp < HARD_MIN_M;
        Mat wayTooFar   = depthTemp > HARD_MAX_M;
        depthTemp.setTo(HARD_MAX_M, wayTooFar);
        depthTemp.setTo(HARD_MIN_M, wayTooClose);

        // Keep only valid pixels
        depthMeters.setTo(0);
        depthTemp.copyTo(depthMeters, validMask);

        /* ---- Perceptual display ----
           Auto-range: stretch the ACTUAL min/max depth in this frame
           across the full 0-255 range so you can see gradients.       */
        double sceneMin = HARD_MIN_M, sceneMax = HARD_MAX_M;
        minMaxLoc(depthMeters, &sceneMin, &sceneMax, NULL, NULL, validMask);

        if (sceneMax <= sceneMin) sceneMax = sceneMin + 0.1;
        // Cap the far end so a distant wall doesn't wash out the room
        if (sceneMax > 4.0) sceneMax = 4.0;

        Mat norm;
        if (useLog) {
            // Logarithmic scaling: human-friendly
            // close objects spread out, far objects compress
            Mat logDepth, logMin, logMax;
            log(depthMeters, logDepth);          // ln(depth)
            log(Mat(1, 1, CV_64F, &sceneMin), logMin);
            log(Mat(1, 1, CV_64F, &sceneMax), logMax);
            double lmin = logMin.at<double>(0, 0);
            double lmax = logMax.at<double>(0, 0);
            norm = (lmax - logDepth) / (lmax - lmin);  // 1=close, 0=far
        } else {
            // Linear scaling
            norm = (sceneMax - depthMeters) / (sceneMax - sceneMin);
        }

        // Clamp and kill invalids
        norm.setTo(0, ~validMask);
        norm.convertTo(depthDisplay, CV_8UC1, 255.0);
        medianBlur(depthDisplay, depthDisplay, 3);

        /* ---- FPS ---- */
        ++frameCount;
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double>(now - lastTime).count();
        if (elapsed >= 1.0) {
            currFps    = frameCount / elapsed;
            frameCount = 0;
            lastTime   = now;
        }

        string modeStr = useLog ? "LOG" : "LIN";
        string fpsTxt  = "FPS: " + to_string(int(currFps)) + " [" + modeStr + "]";
        drawText(frame0, fpsTxt + " Right", Point(10, 25));
        drawText(frame1, fpsTxt + " Left",  Point(10, 25));
        drawText(depthDisplay, fpsTxt + " Depth", Point(10, 25), Scalar(255));

        // Show actual scene range for this frame
        stringstream rs;
        rs << fixed << setprecision(2)
           << "Range: " << sceneMin << "m - " << sceneMax << "m";
        putText(depthDisplay, rs.str(), Point(10, 55),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180), 1);

        // Crosshair readings at 3 zones
        for (int x : { width / 4, width / 2, 3 * width / 4 }) {
            float d = depthMeters.at<float>(height / 2, x);
            if (d >= HARD_MIN_M && d <= HARD_MAX_M) {
                stringstream ss;
                ss << fixed << setprecision(2) << d << "m";
                putText(depthDisplay, ss.str(), Point(x - 20, height / 2),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180), 1);
            }
            circle(depthDisplay, Point(x, height / 2), 3, Scalar(180), -1);
        }

        imshow("Left", frame0);
        imshow("Right", frame1);
        imshow("Metric Depth (m)", depthDisplay);

        char key = (char)waitKey(1);
        if (key == 27) break;
        if (key == 'l' || key == 'L') {
            useLog = !useLog;
            cout << "Switched to " << (useLog ? "LOG" : "LINEAR") << " scaling" << endl;
        }
        if (key == ' ') {
            imwrite("metric_left.png",  frame1);
            imwrite("metric_right.png", frame0);
            imwrite("metric_depth.png", depthDisplay);
            FileStorage fs("metric_depth_raw.xml", FileStorage::WRITE);
            fs << "depth_meters" << depthMeters;
            fs << "scene_min" << sceneMin;
            fs << "scene_max" << sceneMax;
            fs.release();
            cout << "Saved.\n";
        }
    }
    cout << "Releasing cameras..." << endl;
    cam0.release();
    cam1.release();
    destroyAllWindows();
    return 0;
}
