#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <atomic>
#include <csignal>
#include <sys/stat.h>

using namespace cv;
using namespace std;

static atomic<bool> g_running{true};
void onSig(int) { g_running = false; }

string buildPipeline(int sensorId, int capWidth, int capHeight, int dispWidth, int dispHeight, int fps)
{
    stringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensorId
       << " ! video/x-raw(memory:NVMM), width=" << capWidth
       << ", height=" << capHeight << ", format=NV12, framerate=" << fps
       << "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=" << dispWidth
       << ", height=" << dispHeight
       << ", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true";
    return ss.str();
}

int main(int argc, char** argv)
{
    int capW = 640, capH = 480, fps = 30;
    int sensorW = 1640, sensorH = 1232;   // native 4:3 mode, square pixels
    int boardW = 9, boardH = 6;
    float squareMm = 25.0f;

    if (argc > 1) capW     = atoi(argv[1]);
    if (argc > 2) capH     = atoi(argv[2]);
    if (argc > 3) fps      = atoi(argv[3]);
    if (argc > 4) boardW   = atoi(argv[4]);
    if (argc > 5) boardH   = atoi(argv[5]);
    if (argc > 6) squareMm = atof(argv[6]);

    /* Keep 4:3 native sensor mode so pixels are square, even if user
       requests a smaller display/save resolution for the UI. */
    if (capW * 3 != capH * 4) {
        cout << "[WARN] requested " << capW << "x" << capH
             << " is not 4:3; using 640x480 output but 1640x1232 sensor mode" << endl;
    }

    float squareSize = squareMm / 1000.0f;
    Size boardSize(boardW, boardH);
    Size imageSize(capW, capH);

    struct sigaction sa;
    sa.sa_handler = onSig;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    mkdir("calib_images", 0777);

    vector<Point3f> objTemplate;
    for (int y = 0; y < boardSize.height; ++y)
        for (int x = 0; x < boardSize.width; ++x)
            objTemplate.emplace_back(x * squareSize, y * squareSize, 0.0f);

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePointsL, imagePointsR;

    /* ---------- CAMERAS SWAPPED TO MATCH PHYSICAL RIG ----------
       In the stereo viewers:  cam0 (sensor-id=0) = LEFT
                               cam1 (sensor-id=1) = RIGHT
       Wait — user said right is left and left is right in calibration.
       Stereo viewer after fix:  Left window = frame0 (cam0)
                                 Right window = frame1 (cam1)
       So cam0 = physical LEFT, cam1 = physical RIGHT.              */
    VideoCapture camL(buildPipeline(1, sensorW, sensorH, capW, capH, fps), CAP_GSTREAMER);
    usleep(500000);
    VideoCapture camR(buildPipeline(0, sensorW, sensorH, capW, capH, fps), CAP_GSTREAMER);

    if (!camL.isOpened() || !camR.isOpened()) {
        cerr << "Camera open failed\n";
        return -1;
    }

    namedWindow("Left",  WINDOW_AUTOSIZE);
    namedWindow("Right", WINDOW_AUTOSIZE);
    namedWindow("Debug Left",  WINDOW_AUTOSIZE);
    namedWindow("Debug Right", WINDOW_AUTOSIZE);

    Mat frameL, frameR, grayL, grayR;
    int savedPairs = 0;
    int frameIdx = 0;

    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
    TermCriteria subPixCrit(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.001);

    cout << "=== Stereo Calibration ===" << endl;
    cout << "Board: " << boardW << "x" << boardH << " inner corners" << endl;
    cout << "Square size: " << squareMm << " mm" << endl;
    cout << "If your printed board is different, run:" << endl;
    cout << "  ./calibrate_stereo 640 480 30 <boardW> <boardH> <squareMm>" << endl;
    cout << "SPACE = save pair  |  C = calibrate (need 10+)  |  ESC = quit" << endl;
    cout << "---" << endl;

    while (g_running) {
        camL >> frameL;
        camR >> frameR;
        if (frameL.empty() || frameR.empty()) continue;

        cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        /* CLAHE helps detection when auto-exposure differs between cameras */
        Mat eqL, eqR;
        clahe->apply(grayL, eqL);
        clahe->apply(grayR, eqR);

        vector<Point2f> cornersL, cornersR;
        bool foundL = findChessboardCorners(eqL, boardSize, cornersL,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        bool foundR = findChessboardCorners(eqR, boardSize, cornersR,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

        Mat showL = frameL.clone();
        Mat showR = frameR.clone();

        if (foundL) {
            cornerSubPix(grayL, cornersL, Size(11, 11), Size(-1, -1), subPixCrit);
            drawChessboardCorners(showL, boardSize, cornersL, true);
        }
        if (foundR) {
            cornerSubPix(grayR, cornersR, Size(11, 11), Size(-1, -1), subPixCrit);
            drawChessboardCorners(showR, boardSize, cornersR, true);
        }

        string status = "Pairs: " + to_string(savedPairs);
        putText(showL, status, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        putText(showR, status, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        string detectStatus;
        if (foundL && foundR) {
            detectStatus = "READY - press SPACE";
            putText(showL, detectStatus, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
            putText(showR, detectStatus, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        } else {
            detectStatus = "L:" + string(foundL ? "YES" : "NO") + " R:" + string(foundR ? "YES" : "NO");
            putText(showL, detectStatus, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
            putText(showR, detectStatus, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }

        imshow("Left", showL);
        imshow("Right", showR);
        imshow("Debug Left", eqL);
        imshow("Debug Right", eqR);

        if (++frameIdx % 30 == 0) {
            cout << "[" << frameIdx << "] Detection L:" << (foundL ? "YES" : "NO")
                 << " R:" << (foundR ? "YES" : "NO")
                 << " | Saved pairs: " << savedPairs << endl;
        }

        int key = waitKey(10);
        if (key == 27) break;
        if (key == ' ') {
            if (foundL && foundR) {
                objectPoints.push_back(objTemplate);
                imagePointsL.push_back(cornersL);
                imagePointsR.push_back(cornersR);
                ++savedPairs;

                stringstream fnL, fnR;
                fnL << "calib_images/left_"  << setw(3) << setfill('0') << savedPairs << ".png";
                fnR << "calib_images/right_" << setw(3) << setfill('0') << savedPairs << ".png";
                imwrite(fnL.str(), frameL);
                imwrite(fnR.str(), frameR);

                cout << ">>> SAVED pair " << savedPairs << endl;
            } else {
                cout << ">>> SPACE ignored: board not detected in both (L:"
                     << (foundL ? "YES" : "NO") << " R:" << (foundR ? "YES" : "NO") << ")" << endl;
            }
        }

        if ((key == 'c' || key == 'C') && savedPairs >= 5) {
            cout << "\nRunning stereo calibration with " << savedPairs << " pairs..." << endl;

            Mat K1 = initCameraMatrix2D(objectPoints, imagePointsL, imageSize, 0);
            Mat K2 = initCameraMatrix2D(objectPoints, imagePointsR, imageSize, 0);
            Mat D1 = Mat::zeros(8, 1, CV_64F);
            Mat D2 = Mat::zeros(8, 1, CV_64F);
            Mat R, T, E, F;

            double rms = stereoCalibrate(objectPoints, imagePointsL, imagePointsR,
                K1, D1, K2, D2, imageSize, R, T, E, F,
                CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_USE_INTRINSIC_GUESS,
                TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

            cout << "Calibration RMS error: " << rms << " pixels" << endl;

            Mat R1, R2, P1, P2, Q;
            Rect validRoi[2];
            stereoRectify(K1, D1, K2, D2, imageSize, R, T,
                R1, R2, P1, P2, Q,
                CALIB_ZERO_DISPARITY, 1.0, imageSize,
                &validRoi[0], &validRoi[1]);

            Mat mapLx, mapLy, mapRx, mapRy;
            initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2, mapLx, mapLy);
            initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_16SC2, mapRx, mapRy);

            FileStorage fs("stereo_calib.yml", FileStorage::WRITE);
            fs << "image_width"  << capW;
            fs << "image_height" << capH;
            fs << "board_w"      << boardW;
            fs << "board_h"      << boardH;
            fs << "square_mm"    << squareMm;
            fs << "K1" << K1;
            fs << "D1" << D1;
            fs << "K2" << K2;
            fs << "D2" << D2;
            fs << "R"  << R;
            fs << "T"  << T;
            fs << "R1" << R1;
            fs << "R2" << R2;
            fs << "P1" << P1;
            fs << "P2" << P2;
            fs << "Q"  << Q;
            fs << "mapLx" << mapLx;
            fs << "mapLy" << mapLy;
            fs << "mapRx" << mapRx;
            fs << "mapRy" << mapRy;
            fs.release();

            cout << "Saved stereo_calib.yml" << endl;
            cout << "Valid ROI left:  " << validRoi[0] << endl;
            cout << "Valid ROI right: " << validRoi[1] << endl;
        }
    }

    camL.release();
    camR.release();
    destroyAllWindows();
    return 0;
}
