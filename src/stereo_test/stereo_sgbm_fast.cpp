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

void onSignal(int)
{
    g_keepRunning = false;
}

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

int main(int argc, char** argv)
{
    int capW   = 640;
    int capH   = 480;
    int fps    = 30;
    int procW  = 320;
    int procH  = 240;

    if (argc > 1) capW  = atoi(argv[1]);
    if (argc > 2) capH  = atoi(argv[2]);
    if (argc > 3) fps   = atoi(argv[3]);
    procW = capW / 2;
    procH = capH / 2;

    /* ---------- Try to load calibration ---------- */
    Mat mapLx, mapLy, mapRx, mapRy;
    Mat Q_calib;
    double fx_cap = 0.0, baseline = 0.0;
    bool haveCalib = false;

    FileStorage fs("stereo_calib.yml", FileStorage::READ);
    if (fs.isOpened()) {
        fs["mapLx"] >> mapLx;
        fs["mapLy"] >> mapLy;
        fs["mapRx"] >> mapRx;
        fs["mapRy"] >> mapRy;
        fs["Q"] >> Q_calib;

        if (!mapLx.empty() && !mapRx.empty() && !Q_calib.empty()) {
            haveCalib = true;
            fx_cap   = Q_calib.at<double>(2, 3);
            baseline = -1.0 / Q_calib.at<double>(3, 2);
            cout << "=== Loaded stereo_calib.yml ===" << endl;
            cout << "fx=" << fx_cap << " px  baseline=" << baseline << " m" << endl;
        }
        fs.release();
    }

    if (!haveCalib) {
        cout << "WARNING: stereo_calib.yml not found. Using hardcoded fallback." << endl;
        const double BASELINE_M = 0.060;
        const double HFOV_DEG   = 73.0;
        fx_cap   = (capW / 2.0) / tan(HFOV_DEG * CV_PI / 360.0);
        baseline = BASELINE_M;
    }

    /* fx at processing resolution (disparity is computed there) */
    double fx_proc = fx_cap * (procW / (double)capW);
    const double depth_scale = fx_proc * baseline;
    const double HARD_MIN_M = 0.20;
    const double HARD_MAX_M = 10.0;

    struct sigaction sa;
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    cout << "=== StereoSGBM Fast (" << procW << "x" << procH << ") ===" << endl;
    cout << "Capture: " << capW << "x" << capH << " | Stereo: " << procW << "x" << procH << endl;
    cout << "fx@proc=" << fx_proc << "  depth_scale=" << depth_scale << endl;
    cout << "ESC=quit | SPACE=save | L=toggle log/linear | I=toggle info | D=toggle disparity view" << endl;

    VideoCapture cam0(buildPipeline(1, capW, capH, fps), CAP_GSTREAMER);
    usleep(500000);
    VideoCapture cam1(buildPipeline(0, capW, capH, fps), CAP_GSTREAMER);
    if (!cam0.isOpened() || !cam1.isOpened()) {
        cerr << "Camera open failed\n";
        return -1;
    }

    /* SGBM tuned for low-res noisy images */
    int numDisp   = 16 * 6;   // 96 disparities (was 80)
    int blockSize = 7;        // 7x7 blocks (was 5) -> more robust to noise
    int cn        = 1;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, numDisp, blockSize);
    sgbm->setP1(8 * cn * blockSize * blockSize);
    sgbm->setP2(32 * cn * blockSize * blockSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numDisp);
    sgbm->setUniquenessRatio(8);           // slightly relaxed
    sgbm->setSpeckleWindowSize(80);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setPreFilterCap(63);             // helps with brightness diff
    sgbm->setMode(StereoSGBM::MODE_SGBM);

    namedWindow("Left",  WINDOW_AUTOSIZE);
    namedWindow("Right", WINDOW_AUTOSIZE);
    namedWindow("Depth", WINDOW_AUTOSIZE);

    auto lastTime  = chrono::high_resolution_clock::now();
    int frameCount = 0;
    double currFps = 0.0;
    bool useLog    = true;
    bool showInfo  = true;
    bool showDisp  = false;  // toggle raw disparity with 'D'

    Mat frame0, frame1;
    Mat rect0, rect1;
    Mat rectGray0, rectGray1;
    Mat small0, small1, disp16S, dispFloat;
    Mat depthMetersProc(procH, procW, CV_32F);
    Mat depthDisplay(procH, procW, CV_8UC1);
    Mat depthDisplayUp(capH, capW, CV_8UC1);

    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));

    while (g_keepRunning) {
        cam0 >> frame0;
        cam1 >> frame1;
        if (frame0.empty() || frame1.empty()) continue;

        /* Rectify */
        if (haveCalib) {
            remap(frame0, rect0, mapLx, mapLy, INTER_LINEAR);
            remap(frame1, rect1, mapRx, mapRy, INTER_LINEAR);
            cvtColor(rect0, rectGray0, COLOR_BGR2GRAY);
            cvtColor(rect1, rectGray1, COLOR_BGR2GRAY);
        } else {
            rect0 = frame0;
            rect1 = frame1;
            cvtColor(frame0, rectGray0, COLOR_BGR2GRAY);
            cvtColor(frame1, rectGray1, COLOR_BGR2GRAY);
        }

        /* Downscale */
        resize(rectGray0, small0, Size(procW, procH), 0, 0, INTER_LINEAR);
        resize(rectGray1, small1, Size(procW, procH), 0, 0, INTER_LINEAR);

        /* ---------- Preprocess: CLAHE + brightness normalization ---------- */
        clahe->apply(small0, small0);
        clahe->apply(small1, small1);

        Scalar meanL = mean(small0);
        Scalar meanR = mean(small1);
        double diff = meanL[0] - meanR[0];
        if (std::abs(diff) > 2.0) {
            small1.convertTo(small1, CV_8UC1, 1.0, diff);
        }

        /* Stereo matching */
        sgbm->compute(small0, small1, disp16S);
        disp16S.convertTo(dispFloat, CV_32F, 1.0 / 16.0);

        Mat validMaskProc = (dispFloat > 1.0f) & (dispFloat < numDisp);

        /* Metric depth at proc resolution */
        Mat depthTemp = depth_scale / dispFloat;
        Mat badClose = depthTemp < HARD_MIN_M;
        Mat badFar   = depthTemp > HARD_MAX_M;
        depthTemp.setTo(HARD_MAX_M, badFar);
        depthTemp.setTo(HARD_MIN_M, badClose);

        depthMetersProc.setTo(0);
        depthTemp.copyTo(depthMetersProc, validMaskProc);

        /* Count valid pixels for debug */
        int validCount = countNonZero(validMaskProc);
        double validPct = 100.0 * validCount / (procW * procH);

        /* ---------- Display at proc resolution then upscale ---------- */
        double sceneMin = HARD_MIN_M, sceneMax = HARD_MAX_M;
        if (validCount > 0) {
            minMaxLoc(depthMetersProc, &sceneMin, &sceneMax, NULL, NULL, validMaskProc);
        }
        if (sceneMax <= sceneMin) sceneMax = sceneMin + 0.1;
        if (sceneMax > 4.0) sceneMax = 4.0;

        Mat norm;
        if (useLog) {
            Mat logDepth, lm, lx;
            log(depthMetersProc, logDepth);
            log(Mat(1, 1, CV_64F, &sceneMin), lm);
            log(Mat(1, 1, CV_64F, &sceneMax), lx);
            norm = (lx.at<double>(0,0) - logDepth) / (lx.at<double>(0,0) - lm.at<double>(0,0));
        } else {
            norm = (sceneMax - depthMetersProc) / (sceneMax - sceneMin);
        }
        norm.setTo(0, ~validMaskProc);
        norm.convertTo(depthDisplay, CV_8UC1, 255.0);

        if (showDisp) {
            /* Raw disparity view */
            Mat dispNorm;
            normalize(dispFloat, dispNorm, 0, 255, NORM_MINMAX, CV_8UC1, validMaskProc);
            resize(dispNorm, depthDisplayUp, Size(capW, capH), 0, 0, INTER_LINEAR);
            applyColorMap(depthDisplayUp, depthDisplayUp, COLORMAP_JET);
            cvtColor(depthDisplayUp, depthDisplayUp, COLOR_BGR2GRAY); // back to gray for uniform output
        } else {
            /* Depth view */
            resize(depthDisplay, depthDisplayUp, Size(capW, capH), 0, 0, INTER_LINEAR);
            medianBlur(depthDisplayUp, depthDisplayUp, 3);
        }

        /* FPS */
        ++frameCount;
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double>(now - lastTime).count();
        if (elapsed >= 1.0) {
            currFps    = frameCount / elapsed;
            frameCount = 0;
            lastTime   = now;
        }

        if (showInfo) {
            string txt = "FPS:" + to_string(int(currFps));
            putText(frame0, txt, Point(10, 22), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0,255,0), 2);
            putText(frame1, txt, Point(10, 22), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0,255,0), 2);

            stringstream dtxt;
            dtxt << txt << (useLog ? " LOG" : " LIN") << (showDisp ? " DISP" : " DEPTH");
            putText(depthDisplayUp, dtxt.str(), Point(10, 22),
                    FONT_HERSHEY_SIMPLEX, 0.55, Scalar(180), 2);

            stringstream rs;
            rs << fixed << setprecision(2) << sceneMin << "m-" << sceneMax << "m  valid:" << int(validPct) << "%";
            putText(depthDisplayUp, rs.str(), Point(10, capH - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180), 1);
        }

        imshow("Left",  haveCalib ? rect0 : frame0);
        imshow("Right", haveCalib ? rect1 : frame1);
        imshow("Depth", depthDisplayUp);

        if (getWindowProperty("Depth", WND_PROP_VISIBLE) == 0) break;

        char key = (char)waitKey(1);
        if (key == 27) break;
        if (key == 'l' || key == 'L') useLog = !useLog;
        if (key == 'i' || key == 'I') showInfo = !showInfo;
        if (key == 'd' || key == 'D') showDisp = !showDisp;
        if (key == ' ') {
            imwrite("fast_left.png",  haveCalib ? rect0 : frame0);
            imwrite("fast_right.png", haveCalib ? rect1 : frame1);
            imwrite("fast_depth.png", depthDisplayUp);
            FileStorage xfs("fast_depth_raw.xml", FileStorage::WRITE);
            xfs << "depth" << depthMetersProc;
            xfs.release();
            cout << "Saved.\n";
        }
    }

    cout << "Releasing cameras..." << endl;
    cam0.release();
    cam1.release();
    destroyAllWindows();
    cout << "Done." << endl;
    return 0;
}
