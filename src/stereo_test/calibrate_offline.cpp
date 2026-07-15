#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glob.h>

using namespace cv;
using namespace std;

vector<string> globFiles(const string& pattern) {
    vector<string> files;
    glob_t glob_result;
    if (glob(pattern.c_str(), 0, NULL, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i)
            files.push_back(glob_result.gl_pathv[i]);
    }
    globfree(&glob_result);
    sort(files.begin(), files.end());
    return files;
}

bool saneCalibration(const Mat& K, const Mat& P, Size imageSize) {
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    double pfx = P.at<double>(0,0);
    double pcx = P.at<double>(0,2);
    double pcy = P.at<double>(1,2);

    if (fx < 100 || fx > 2000) { cerr << "Bad fx=" << fx << "\n"; return false; }
    if (fy < 100 || fy > 2000) { cerr << "Bad fy=" << fy << "\n"; return false; }
    if (cx < 0 || cx > imageSize.width * 1.5) { cerr << "Bad cx=" << cx << "\n"; return false; }
    if (cy < 0 || cy > imageSize.height * 1.5) { cerr << "Bad cy=" << cy << "\n"; return false; }
    if (pfx < 0) { cerr << "Bad P fx=" << pfx << " (negative)\n"; return false; }
    if (pcx < 0 || pcx > imageSize.width * 2) { cerr << "Bad P cx=" << pcx << "\n"; return false; }
    if (pcy < 0 || pcy > imageSize.height * 2) { cerr << "Bad P cy=" << pcy << "\n"; return false; }
    return true;
}

int main(int argc, char** argv)
{
    int capW = 640, capH = 480;
    int boardW = 7, boardH = 9;
    float squareMm = 20.0f;

    if (argc > 1) capW   = atoi(argv[1]);
    if (argc > 2) capH   = atoi(argv[2]);
    if (argc > 3) boardW = atoi(argv[3]);
    if (argc > 4) boardH = atoi(argv[4]);
    if (argc > 5) squareMm = atof(argv[5]);

    float squareSize = squareMm / 1000.0f;
    Size boardSize(boardW, boardH);
    Size imageSize(capW, capH);

    vector<string> leftFiles  = globFiles("calib_images/left_*.png");
    vector<string> rightFiles = globFiles("calib_images/right_*.png");

    if (leftFiles.empty() || leftFiles.size() != rightFiles.size()) {
        cerr << "Error: mismatch or no images in calib_images/\n";
        return 1;
    }

    cout << "Found " << leftFiles.size() << " pairs. Detecting corners...\n";

    vector<Point3f> objTemplate;
    for (int y = 0; y < boardSize.height; ++y)
        for (int x = 0; x < boardSize.width; ++x)
            objTemplate.emplace_back(x * squareSize, y * squareSize, 0.0f);

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePointsL, imagePointsR;
    TermCriteria subPixCrit(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.001);
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));

    for (size_t i = 0; i < leftFiles.size(); ++i) {
        Mat imgL = imread(leftFiles[i]);
        Mat imgR = imread(rightFiles[i]);
        if (imgL.empty() || imgR.empty()) continue;

        Mat grayL, grayR;
        cvtColor(imgL, grayL, COLOR_BGR2GRAY);
        cvtColor(imgR, grayR, COLOR_BGR2GRAY);

        Mat eqL, eqR;
        clahe->apply(grayL, eqL);
        clahe->apply(grayR, eqR);

        vector<Point2f> cornersL, cornersR;
        bool foundL = findChessboardCorners(eqL, boardSize, cornersL,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        bool foundR = findChessboardCorners(eqR, boardSize, cornersR,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

        if (foundL && foundR) {
            cornerSubPix(grayL, cornersL, Size(11, 11), Size(-1, -1), subPixCrit);
            cornerSubPix(grayR, cornersR, Size(11, 11), Size(-1, -1), subPixCrit);
            objectPoints.push_back(objTemplate);
            imagePointsL.push_back(cornersL);
            imagePointsR.push_back(cornersR);
            cout << "  Pair " << (i+1) << "/" << leftFiles.size() << " -> OK\n";
        } else {
            cout << "  Pair " << (i+1) << "/" << leftFiles.size() << " -> FAIL (L:" 
                 << (foundL ? "Y" : "N") << " R:" << (foundR ? "Y" : "N") << ")\n";
        }
    }

    if (objectPoints.size() < 5) {
        cerr << "Need at least 5 valid pairs, got " << objectPoints.size() << "\n";
        return 1;
    }

    /* ---------- Better initial guess ---------- */
    double f0 = max(capW, capH) * 0.9;  // ~500 for 640x480
    Mat K1 = (Mat_<double>(3,3) << f0, 0, capW/2.0,
                                    0, f0, capH/2.0,
                                    0,  0,      1.0);
    Mat K2 = K1.clone();
    Mat D1 = Mat::zeros(5, 1, CV_64F);
    Mat D2 = Mat::zeros(5, 1, CV_64F);
    Mat R, T, E, F;

    cout << "\nRunning stereo calibration with " << objectPoints.size() << " pairs...\n";

    int calibFlags = CALIB_FIX_ASPECT_RATIO
                   | CALIB_ZERO_TANGENT_DIST
                   | CALIB_FIX_K3
                   | CALIB_FIX_K4
                   | CALIB_FIX_K5
                   | CALIB_FIX_K6
                   | CALIB_USE_INTRINSIC_GUESS;

    double rms = stereoCalibrate(objectPoints, imagePointsL, imagePointsR,
        K1, D1, K2, D2, imageSize, R, T, E, F,
        calibFlags,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    cout << "Calibration RMS error: " << rms << " pixels\n";

    if (rms > 2.0) {
        cerr << "\nWARNING: RMS > 2.0 pixels. Calibration is poor.\n";
        cerr << "Common causes:\n";
        cerr << "  - Board too small in images (should fill ~1/3 of frame)\n";
        cerr << "  - Not enough pose variation (tilt, distance, position)\n";
        cerr << "  - Motion blur or out-of-focus board\n";
        cerr << "  - Wrong square size (measure with ruler!)\n";
    }

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    stereoRectify(K1, D1, K2, D2, imageSize, R, T,
        R1, R2, P1, P2, Q,
        CALIB_ZERO_DISPARITY, 1.0, imageSize,
        &validRoi[0], &validRoi[1]);

    /* Sanity checks */
    if (!saneCalibration(K1, P1, imageSize) || !saneCalibration(K2, P2, imageSize)) {
        cerr << "\nCALIBRATION FAILED SANITY CHECKS.\n";
        cerr << "K1 = " << K1 << "\nK2 = " << K2 << "\nP1 = " << P1 << "\nP2 = " << P2 << "\n";
        return 1;
    }

    if (validRoi[0].width < 100 || validRoi[0].height < 100) {
        cerr << "\nWARNING: valid ROI is very small: " << validRoi[0] << "\n";
        cerr << "This means rectification will crop most of the image.\n";
    }

    Mat mapLx, mapLy, mapRx, mapRy;
    initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2, mapLx, mapLy);
    initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_16SC2, mapRx, mapRy);

    FileStorage cfs("stereo_calib.yml", FileStorage::WRITE);
    cfs << "image_width"  << capW;
    cfs << "image_height" << capH;
    cfs << "board_w"      << boardW;
    cfs << "board_h"      << boardH;
    cfs << "square_mm"    << squareMm;
    cfs << "rms_pixels"   << rms;
    cfs << "K1" << K1;
    cfs << "D1" << D1;
    cfs << "K2" << K2;
    cfs << "D2" << D2;
    cfs << "R"  << R;
    cfs << "T"  << T;
    cfs << "R1" << R1;
    cfs << "R2" << R2;
    cfs << "P1" << P1;
    cfs << "P2" << P2;
    cfs << "Q"  << Q;
    cfs << "mapLx" << mapLx;
    cfs << "mapLy" << mapLy;
    cfs << "mapRx" << mapRx;
    cfs << "mapRy" << mapRy;
    cfs.release();

    cout << "\nSaved stereo_calib.yml\n";
    cout << "K1: fx=" << K1.at<double>(0,0) << " fy=" << K1.at<double>(1,1)
         << " cx=" << K1.at<double>(0,2) << " cy=" << K1.at<double>(1,2) << "\n";
    cout << "K2: fx=" << K2.at<double>(0,0) << " fy=" << K2.at<double>(1,1)
         << " cx=" << K2.at<double>(0,2) << " cy=" << K2.at<double>(1,2) << "\n";
    cout << "Baseline from T: " << norm(T) << " m\n";
    cout << "Valid ROI left:  " << validRoi[0] << "\n";
    cout << "Valid ROI right: " << validRoi[1] << "\n";
    return 0;
}
