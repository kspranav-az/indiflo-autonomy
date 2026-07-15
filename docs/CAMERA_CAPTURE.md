# Camera Capture Investigation

## Problem

The Jetson CSI camera (`/dev/video0`) outputs **RG10 Bayer RAW** (10-bit, RGGB pattern, 3280×2464 native resolution). The goal is to capture frames, debayer to RGB, run MiDaS depth estimation via TensorRT, and publish ROS2 Image messages.

## What Works

- **V4L2 raw capture** works — `VIDIOC_DQBUF` successfully returns frames at ~5–6 FPS
- **TensorRT inference** works — the MiDaS engine loads and produces depth output
- **ROS2 topics publish** — `/camera/left/image_raw`, `/camera/left/depth_raw`, `/camera/left/camera_info` are all live
- **TF and point cloud pipeline** works — RViz shows the frame tree, point cloud, and laser scan

## What Does NOT Work

### GStreamer + nvarguscamerasrc (BLOCKED)

`nvarguscamerasrc` can display via `nvgstcapture-1.0` and `nveglglessink`, but **any pipeline ending in `appsink` fails silently** — `pull_sample()` blocks forever. The `nvvidconv` element outputs DMA-BUF memory that `appsink` cannot map on this JetPack version.

Tried:
- `nvarguscamerasrc ! nvvidconv ! video/x-raw,format=I420 ! appsink`
- `nvarguscamerasrc ! nvvidconv ! video/x-raw,format=NV12 ! appsink`
- `nvarguscamerasrc ! nvvidconv ! video/x-raw,format=RGBA ! appsink`
- With/without `memory:NVMM`, `copy-output=1`, `emit-signals=true`, `max-buffers=1`, `drop=true`

**Result:** No frames ever reach CPU memory via GStreamer `appsink`. This blocks the NVIDIA ISP path entirely.

### V4L2 Exposure/Gain Controls (NO-OP for raw)

`VIDIOC_S_EXT_CTRLS` succeeds for `exposure` and `gain`, but the **raw Bayer output does not change**. These controls only affect the ISP pipeline (via `nvarguscamerasrc`), not raw V4L2 capture.

`frame_rate` control **does** work and was raised from 2 Hz (`2000000`) to max (`21000000`) to restore FPS.

## Bit Packing

- Format: `V4L2_PIX_FMT_SRGGB10` (fourcc `'RG10'`)
- `sizeimage = 16163840 = 3280 * 2464 * 2` bytes
- Each pixel is a 16-bit word with 10 valid bits in the **upper bits (15:6)**
- Correct conversion to 8-bit: `value / 64.0` (extracts full 10-bit range, clips to 255)
  - `1.0/256.0` → too dark (drops 2 LSBs)
  - `1.0/4.0` → all white (over-saturates)
  - `1.0/64.0` → correct brightness

## Bayer Pattern (CONFIRMED)

Analyzed raw pixel values from captured frames. The pattern is definitively **RGGB**:

| Position | Value (typical) | Expected (RGGB) |
|----------|----------------|-----------------|
| (0,0)    | ~100           | R               |
| (0,1)    | ~180           | G               |
| (1,0)    | ~250           | G               |
| (1,1)    | ~200           | B               |

Both OpenCV `cvtColor` and manual bilinear demosaic produce identical results, confirming the pattern and debayer implementation are correct.

## Color Issue (ROOT CAUSE)

**Raw Bayer capture bypasses the NVIDIA ISP completely.** The ISP normally applies:
1. Black level correction
2. White balance
3. Color correction matrix (CCM)
4. Gamma / tone mapping
5. Denoising

Without the ISP, the sensor's native color response is visible. The raw RGGB data has a **strong green cast**:

- Center crop averages: **B≈105, G≈126, R≈110**
- Even after gray-world white balance, the image retains a green tint because the sensor's spectral response is green-heavy

Software corrections applied:
- **Gray-world white balance** — scales all channels to equal average
- **Linear color correction matrix** — attempts to remap raw RGB toward sRGB

Current CCM (aggressive):
```
[ 1.60  -0.35  -0.25 ]
[-0.20   1.00  -0.20 ]
[-0.20  -0.40   1.60 ]
```

This pushes G down significantly and boosts R/B, but results are still not natural. A sensor-specific CCM (from the camera module datasheet or NVIDIA ISP tuning file) is needed for proper color reproduction.

## Graininess

Raw Bayer images are inherently noisy because:
- No ISP denoising is applied
- CPU demosaic (OpenCV bilinear) amplifies noise
- 10→8 bit conversion preserves noise

The `nvgstcapture-1.0` image looks smooth because the NVIDIA ISP applies temporal and spatial noise reduction.

## Performance

At 3280×2464:
- `convertTo` (10→8 bit): ~2 ms
- OpenCV bilinear demosaic: ~8 ms
- `cv::transform` (WB + CCM): ~5 ms
- TensorRT inference (256×256): ~30 ms
- Total per frame: ~50–60 ms → **~5–6 FPS**

## Open Questions

1. Can a sensor-specific CCM be extracted from NVIDIA ISP tuning files?
2. Is there a way to run `nvarguscamerasrc` → `appsink` on this JetPack version?
3. Would downsampling the Bayer image before demosaic (e.g., 820×616) improve FPS enough to add CPU denoising?
