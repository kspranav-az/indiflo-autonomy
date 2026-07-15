#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <image.png>\n";
        return 1;
    }

    Mat img = imread(argv[1]);
    if (img.empty()) {
        cerr << "Failed to load " << argv[1] << "\n";
        return 1;
    }

    // Resize huge images (e.g. 4032x3024 phone photos) for speed
    if (img.cols > 2000 || img.rows > 1500) {
        Mat tmp;
        double scale = min(1280.0 / img.cols, 960.0 / img.rows);
        resize(img, tmp, Size(), scale, scale, INTER_AREA);
        img = tmp;
    }

    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    cout << "Image: " << img.cols << "x" << img.rows << "\n";
    cout << string(40, '-') << "\n";

    vector<pair<int,int>> candidates = {
        {5,7}, {5,8}, {5,9},
        {6,7}, {6,8}, {6,9},
        {7,7}, {7,8}, {7,9}, {7,10},
        {8,6}, {8,7}, {8,8}, {8,9}, {8,10},
        {9,6}, {9,7}, {9,8}, {9,9}, {9,10},
        {10,7}, {10,8}, {10,9},
    };

    bool found_any = false;
    for (auto [w, h] : candidates) {
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, Size(w,h), corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        if (found) {
            found_any = true;
            cout << "  " << w << "x" << h << " inner corners -> FOUND  <<< USE THIS\n";
        } else {
            cout << "  " << w << "x" << h << " inner corners -> no\n";
        }
    }

    cout << string(40, '-') << "\n";
    if (!found_any) {
        cout << "No size matched.\n";
        cout << "Common causes:\n";
        cout << "  - Board is cut off at image edge\n";
        cout << "  - Image is too blurry\n";
        cout << "  - Board is wrinkled / not flat\n";
        cout << "  - Print quality poor (squares bleeding)\n";
    } else {
        cout << "\nRun calibration with the size that says FOUND.\n";
        cout << "Example: ./calibrate_stereo 640 480 30 7 9 20\n";
    }
    return 0;
}
