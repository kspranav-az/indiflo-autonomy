#!/usr/bin/env python3
"""Generate an A4 checkerboard pattern for stereo camera calibration.

Usage:
    python3 generate_checkerboard.py [ROWS] [COLS] [--margin_mm M] [--output NAME]

Defaults are tuned for A4 (210 x 297 mm) with a decent border:
    python3 generate_checkerboard.py 8 5

The checkerboard pattern uses the internal corner count (rows x cols), which
is what OpenCV's findChessboardCorners expects. Print at 100% scale (no fit /
no shrink), attach to a rigid flat backing (cardboard / acrylic), and measure
one square edge with a ruler. Pass that measured size to the calibration tool.
"""

import argparse
import os
from math import floor

try:
    from reportlab.pdfgen import canvas
    from reportlab.lib.pagesizes import A4
    HAS_REPORTLAB = True
except ImportError:
    HAS_REPORTLAB = False


def a4_square_size(rows, cols, margin_mm):
    """Compute the largest square size that fits A4 with the given margin."""
    a4_w, a4_h = 210.0, 297.0  # A4 portrait, mm
    usable_w = a4_w - 2.0 * margin_mm
    usable_h = a4_h - 2.0 * margin_mm
    sq_w = usable_w / (cols + 1)
    sq_h = usable_h / (rows + 1)
    # Round down to nearest 0.5 mm so the printer has an easier job
    return floor(min(sq_w, sq_h) * 2.0) / 2.0


def generate_svg(rows, cols, square_mm, margin_mm, output_path):
    """Write a printable checkerboard SVG."""
    a4_w, a4_h = 210.0, 297.0
    pattern_w = square_mm * (cols + 1)
    pattern_h = square_mm * (rows + 1)
    ox = (a4_w - pattern_w) / 2.0
    oy = (a4_h - pattern_h) / 2.0

    # mm to pixels at 300 DPI for crisp printing
    dpi = 300.0
    mm_to_px = dpi / 25.4
    scale = mm_to_px
    w_px = int(round(a4_w * scale))
    h_px = int(round(a4_h * scale))

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w_px}px" height="{h_px}px" '
               f'viewBox="0 0 {a4_w:.2f} {a4_h:.2f}">')
    svg.append(f'  <rect x="0" y="0" width="{a4_w:.2f}" height="{a4_h:.2f}" fill="white"/>')

    # Draw black squares. Use (cols+1) x (rows+1) checker pattern,
    # anchored so a black square is at the top-left and bottom-right.
    for y in range(rows + 1):
        for x in range(cols + 1):
            if (x + y) % 2 == 0:
                sx = ox + x * square_mm
                sy = oy + y * square_mm
                svg.append(f'  <rect x="{sx:.2f}" y="{sy:.2f}" '
                           f'width="{square_mm:.2f}" height="{square_mm:.2f}" fill="black"/>')

    svg.append('</svg>')

    with open(output_path, 'w') as f:
        f.write('\n'.join(svg))


def generate_pdf(rows, cols, square_mm, margin_mm, output_path):
    """Write a printable checkerboard PDF using reportlab."""
    if not HAS_REPORTLAB:
        raise RuntimeError(
            "reportlab is not installed; cannot generate PDF. "
            "Run: pip3 install --user reportlab")

    a4_w, a4_h = A4  # points: 595.27 x 841.89 (~210x297 mm)
    mm_to_pt = a4_w / 210.0  # ~2.8346 pt/mm

    pattern_w = square_mm * (cols + 1)
    pattern_h = square_mm * (rows + 1)
    ox = (210.0 - pattern_w) / 2.0
    oy = (297.0 - pattern_h) / 2.0

    c = canvas.Canvas(output_path, pagesize=A4)
    # White background is implicit on A4 canvas, but draw it explicitly.
    c.setFillColorRGB(1, 1, 1)
    c.rect(0, 0, a4_w, a4_h, fill=1, stroke=0)

    c.setFillColorRGB(0, 0, 0)
    for y in range(rows + 1):
        for x in range(cols + 1):
            if (x + y) % 2 == 0:
                # PDF y grows upward; our pattern is measured from top-left down.
                sx = ox + x * square_mm
                sy_top = oy + y * square_mm
                sy_pdf = 297.0 - sy_top - square_mm
                c.rect(sx * mm_to_pt, sy_pdf * mm_to_pt,
                       square_mm * mm_to_pt, square_mm * mm_to_pt,
                       fill=1, stroke=0)

    c.showPage()
    c.save()


def generate_calibration_info(rows, cols, square_mm, output_path):
    """Write a sidecar text file with capture + command instructions."""
    text = f"""Checkerboard for A4 stereo calibration
========================================
Internal corner rows: {rows}
Internal corner cols: {cols}
Square size: {square_mm:.1f} mm  <-- MEASURE THIS WITH A RULER AND UPDATE IF NEEDED

The calibration tools expect arguments as:  boardW(cols)  boardH(rows)  square_mm
So the commands below use:  {cols} {rows} {square_mm:.1f}

After printing at 100% scale (no scaling/fit-to-page):
1. Attach the printed page to a FLAT, RIGID backing (cardboard, acrylic, foam board).
2. Verify the board is not curved, wrinkled, or glossy (matte print preferred).
3. Measure the side length of one square with a ruler and update SQUARE_MM below.

Capture:
--------
cd /workspaces/ros2_ws/src/stereo_test/build
./calibrate_stereo 640 480 30 {cols} {rows} {square_mm:.1f}

In the GUI:
  - Show the board to both cameras at the SAME time.
  - Press SPACE when both left and right views show green corners.
  - Capture 15-25 pairs with the board filling ~1/3 to 1/2 of the image.
  - Vary distance, angle, and position across the frame (edges/corners too).
  - Avoid motion blur; hold steady when saving.
  - Press 'C' to run calibration after capturing enough pairs.

Offline calibration (if you already saved images to calib_images/):
--------------------------------------------------------------------
cd /workspaces/ros2_ws/src/stereo_test/build
./calibrate_offline 640 480 {cols} {rows} {square_mm:.1f}

Copy results into the ROS / OpenVINS configs:
---------------------------------------------
cp stereo_calib.yml ../../stereo_depth_ros2/cfg/stereo_calib.yml
# Also hand-update src/stereo_depth_ros2/config/openvins/cam_chain.yaml
# with the new intrinsics, distortion, and cam0->cam1 transform from stereo_calib.yml.
"""
    txt_path = output_path.replace('.svg', '.txt')
    with open(txt_path, 'w') as f:
        f.write(text)
    return txt_path


def main():
    parser = argparse.ArgumentParser(description='Generate an A4 checkerboard for stereo calibration.')
    parser.add_argument('rows', type=int, nargs='?', default=8,
                        help='Number of internal corner rows (default 8)')
    parser.add_argument('cols', type=int, nargs='?', default=5,
                        help='Number of internal corner cols (default 5)')
    parser.add_argument('--margin_mm', type=float, default=15.0,
                        help='Minimum page margin in mm (default 15)')
    parser.add_argument('--output', type=str, default='checkerboard_a4.svg',
                        help='Output SVG filename (default checkerboard_a4.svg)')
    parser.add_argument('--pdf', action='store_true',
                        help='Also generate a PDF version of the checkerboard')
    parser.add_argument('--pdf-output', type=str, default=None,
                        help='PDF output filename (default replaces .svg with .pdf)')
    args = parser.parse_args()

    if args.rows < 3 or args.cols < 3:
        raise ValueError('Need at least 3x3 internal corners for stable calibration.')

    square_mm = a4_square_size(args.rows, args.cols, args.margin_mm)
    if square_mm < 15.0:
        print(f"WARNING: square size is only {square_mm:.1f} mm; "
              "small squares are harder to detect. Consider fewer rows/cols.")

    output_path = os.path.abspath(args.output)
    generate_svg(args.rows, args.cols, square_mm, args.margin_mm, output_path)
    txt_path = generate_calibration_info(args.rows, args.cols, square_mm, output_path)

    pdf_path = None
    if args.pdf:
        pdf_path = os.path.abspath(args.pdf_output) if args.pdf_output else output_path.replace('.svg', '.pdf')
        generate_pdf(args.rows, args.cols, square_mm, args.margin_mm, pdf_path)

    print(f"Generated: {output_path}")
    print(f"Info file: {txt_path}")
    if pdf_path:
        print(f"PDF:       {pdf_path}")
    print(f"Pattern: {args.rows}x{args.cols} internal corners, {square_mm:.1f} mm squares, "
          f"{args.margin_mm:.1f} mm margins")
    print(f"Open the SVG/PDF and print at 100% / actual size (disable 'fit to page').")
    print(f"Then run: ./calibrate_stereo 640 480 30 {args.cols} {args.rows} {square_mm:.1f}")


if __name__ == '__main__':
    main()
