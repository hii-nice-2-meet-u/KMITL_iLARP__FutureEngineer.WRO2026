# Vision System

Detects and classifies the red and green traffic markers used in the Obstacle
Challenge, and reports each detection's position and relative bearing to the
navigation layer.

**Source code:** [`code/modules/camera`](../../../../code/modules/camera)
· Capture: [`camera_module.cpp`](../../../../code/modules/camera/camera_module.cpp)
· Processing: [`camera_processor.cpp`](../../../../code/modules/camera/camera_processor.cpp)
· Shared types: [`camera_struct.hpp`](../../../../code/modules/camera/camera_struct.hpp)

---

## 1. Responsibilities

The vision subsystem is split into two independent concerns:

- **`CameraModule`** — owns the camera hardware and runs a background capture
  thread. It never processes pixels; it only produces timestamped frames.
- **`CameraProcessor`** — a stateless pipeline that turns one frame into a list
  of detected objects. Because it holds no state, it is trivial to unit-test and
  to run on recorded frames offline.

This separation means the capture rate and the processing rate are decoupled:
the newest frame is always available, and the processor consumes frames when it
is ready rather than blocking the camera.

---

## 2. Capture pipeline (`CameraModule`)

The camera is a Raspberry Pi Camera Module 3, accessed through
**libcamera** via the **LCCV** wrapper (`lccv::PiCamera`).

Default configuration (constructor defaults in `camera_module.hpp`):

| Parameter | Default | Notes |
|-----------|---------|-------|
| `video_width` | 640 px | Square capture (see below) |
| `video_height` | 640 px | |
| `framerate` | 90 fps | Capture target |
| `awb_gain_r` | 1.4 | Manual red white-balance gain |
| `awb_gain_b` | 2.6 | Manual blue white-balance gain |

Key design points, all present in `camera_module.cpp`:

- **Dedicated capture thread.** `start()` launches `capture_loop()` on its own
  thread. Continuous autofocus is enabled once at start
  (`AfMode = AfModeContinuous`).
- **180° rotation.** Every frame is flipped on both axes (`cv::flip(frame,
  frame, -1)`) to compensate for the camera being mounted upside-down on the
  chassis.
- **Ring buffer, newest-wins.** Frames are pushed into a
  `RingBuffer<TimedFrameData, 30>`. Consumers call either:
  - `get_latest()` — non-blocking, returns the most recent frame, or
  - `wait_for_frame()` — blocks until a *new* frame arrives (uses a
    `condition_variable` and a frame-sequence counter so the same frame is never
    processed twice).
- **Timestamping.** Each frame is tagged with a `steady_clock` timestamp in
  microseconds at capture time, so downstream fusion can align camera and LiDAR
  data in time.

> **Manual white balance is deliberate.** Auto white balance shifts the hue of
> the red/green markers as lighting changes, which would break the fixed HSV
> thresholds below. Fixing `awb_gain_r`/`awb_gain_b` keeps marker colors stable.
> The tuning tools in [`code/app/adjust_white`](../../../../code/app/adjust_white) and
> [`code/app/adjust_HSV`](../../../../code/app/adjust_HSV) exist to find these values
> for a given venue.

---

## 3. Processing pipeline (`CameraProcessor`)

```
Frame (BGR)
   │
   ▼  cvtColor BGR → HSV
HSV image
   │
   ▼  inRange (red ×2 ranges, green ×1)
Binary masks (red, green)
   │
   ▼  morphology OPEN then CLOSE (4×4 rect kernel)
Cleaned masks
   │
   ▼  findContours (external only)
Contours
   │
   ▼  geometric validation (area / size / aspect / fill)
Valid objects
   │
   ▼  bottom-center + bearing estimation
ProcessedCameraData (list of CameraObject)
```

### 3.1 Color thresholds (HSV)

Defined in `camera_processor.hpp`. Red spans two ranges because its hue wraps
around the 0°/180° boundary of the OpenCV hue circle:

| Color | Lower HSV | Upper HSV |
|-------|-----------|-----------|
| Red (range 1) | (0, 70, 50) | (10, 255, 255) |
| Red (range 2) | (170, 70, 50) | (179, 255, 255) |
| Green | (50, 95, 60) | (85, 200, 200) |

The two red masks are OR-ed together into a single red mask.

### 3.2 Morphological filtering

A single 4×4 rectangular kernel is applied as **OPEN** (remove small
noise specks) followed by **CLOSE** (fill small holes inside a marker), to each
color mask independently.

### 3.3 Contour validation

Contours are extracted with `RETR_EXTERNAL` (outer boundaries only). Each
candidate must pass **all** of the following filters (constants in
`camera_processor.hpp`) to be accepted as a marker:

| Filter | Condition | Value |
|--------|-----------|-------|
| Minimum area | `contourArea ≥` | 650 px² |
| Minimum width | `bbox.width ≥` | 15 px |
| Minimum height | `bbox.height ≥` | 25 px |
| Aspect ratio | `min ≤ w/h ≤ max` | 0.5 … 1.5 |
| Fill ratio | `area / (w·h) ≥` | 0.7 |

The aspect-ratio and fill-ratio checks together reject elongated or hollow
shapes (walls, reflections, floor lines), keeping roughly rectangular, solid
blobs — which is what an upright marker looks like from the car.

### 3.4 Object output

For every valid contour a `CameraObject` is produced with:

- `color` — `Red` or `Green`
- `bounding_box` — the pixel-space rectangle
- `bottom_center` — `(x + w/2, y + h)`, i.e. the mid-point of the bottom edge.
  The **bottom** edge is used (not the centroid) because that is where the
  marker meets the floor, which is the most stable reference for estimating how
  far ahead the marker is.
- `bearing_rad` — see below.

### 3.5 Bearing estimation

The horizontal bearing to a marker is computed from a pinhole-camera model
using the bottom-center x-coordinate:

```
bearing = atan( (pixel_x − cx) / fx )
```

with intrinsic parameters (in `camera_processor.hpp`):

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `fx` | 1268.97425 | Focal length in pixels (x) |
| `cx` | 317.980325 | Principal point x (≈ image center for 640 px) |

A positive bearing means the marker is to the right of the optical axis, a
negative bearing means to the left. This single scalar is what the navigation
layer needs to decide which side to pass a marker on.

---

## 4. Development & calibration tools

These live in [`code/app`](../../../../code/app) and share the same modules:

| Tool | Purpose |
|------|---------|
| [`test_camera`](../../../../code/app/test_camera) | Capture and save frames; validate the module end-to-end |
| [`adjust_HSV`](../../../../code/app/adjust_HSV) | Interactively tune the red/green HSV thresholds |
| [`adjust_white`](../../../../code/app/adjust_white) | Tune the manual white-balance gains |
| [`camera_tuner`](../../../../code/app/camera_tuner) | General camera-parameter tuning |

---

## 5. Decision log (rubric criterion 4)

Decisions that are already visible in the code:

- **BGR→HSV before thresholding** — HSV separates hue from brightness, so a
  single hue band tolerates lighting changes far better than an RGB box.
- **Two red ranges** — required by the hue wrap-around; a single range cannot
  capture red.
- **Bottom-center as the reference point** — most stable ground contact for
  distance/bearing, versus a centroid that moves as the marker is partially
  occluded.
- **Fixed white balance** — trades auto-adaptability for color stability, which
  the fixed HSV thresholds depend on.

The current calibration values are the constants in
`code/modules/camera/camera_processor.hpp`: `fx = 1418.29334` and
`cx = 973.219296`, documented for the 1920×1080 capture geometry. The
repository does not include the calibration procedure, a labelled test set,
measured detection accuracy, or a failure log. The 90 fps / 640×640 capture
configuration is the current `CameraModule` default and should be retained
only after confirming that the deployed camera mode and processing budget match
the hardware.
