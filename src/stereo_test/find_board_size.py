#!/usr/bin/env python3
"""Test a saved calibration image against common chessboard sizes."""
import cv2
import sys

# Common board sizes to try (inner corners = intersections between black squares)
CANDIDATES = [
    (5, 7), (5, 8), (5, 9),
    (6, 7), (6, 8), (6, 9),
    (7, 7), (7, 8), (7, 9), (7, 10),
    (8, 6), (8, 7), (8, 8), (8, 9), (8, 10),
    (9, 6), (9, 7), (9, 8), (9, 9), (9, 10),
    (10, 7), (10, 8), (10, 9),
]

def test_image(path):
    img = cv2.imread(path)
    if img is None:
        print(f"ERROR: Cannot load {path}")
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"Image: {img.shape[1]}x{img.shape[0]}")
    print("-" * 40)

    found_any = False
    for w, h in CANDIDATES:
        found, corners = cv2.findChessboardCorners(
            gray, (w, h),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH |
                  cv2.CALIB_CB_NORMALIZE_IMAGE |
                  cv2.CALIB_CB_FAST_CHECK
        )
        status = "FOUND" if found else "no"
        if found:
            found_any = True
            print(f"  {w}x{h} inner corners -> {status}  <<< USE THIS")
        else:
            print(f"  {w}x{h} inner corners -> {status}")

    print("-" * 40)
    if not found_any:
        print("No size matched. Possible causes:")
        print("  - Board is cut off at image edge")
        print("  - Image is too blurry")
        print("  - Board is not flat (wrinkled paper)")
        print("  - Print quality is poor (squares bleeding together)")
    else:
        print("\nRun calibration with the size that says 'FOUND' above.")
        print("Example: ./calibrate_stereo 640 480 30 7 9 20")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <image.png>")
        sys.exit(1)
    test_image(sys.argv[1])
