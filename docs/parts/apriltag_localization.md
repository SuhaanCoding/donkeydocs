# AprilTag Landmark Localization

The `apriltag_localizer` parts estimate the vehicle's pose `(x, y, theta)` on
a course from printed AprilTag markers, and report when the physical course no
longer matches the surveyed map (a tag was added, moved, or removed). They are
camera-only: no GPS, lidar, or encoder is required, though odometry improves
the estimate when available.

The pose comes from a planar particle filter over range/bearing observations;
a 500-particle numpy filter runs in roughly a tenth of a millisecond per
update, so it comfortably holds camera rate on a Raspberry Pi.

---------------

## Requirements

- Printed [tag36h11](https://github.com/AprilRobotics/apriltag-imgs) markers.
  Print at 16 cm black-square size or larger, matte paper, flat backing.
  **Measure the printed black square with a ruler** — printers rescale
  silently, and every range estimate scales with this number.
- Camera intrinsics `(fx, fy, cx, cy)` from a checkerboard calibration
  (`cv2.calibrateCamera`) or your camera's factory calibration.
- The optional detector dependency:

```bash
pip install donkeycar[apriltag]
# or directly:
pip install pupil-apriltags
```

## Survey the map

Choose a fixed world frame (for example: one room corner is the origin,
positive x runs along a wall) and tape-measure every tag's centre position
and outward facing yaw into a JSON map:

```json
{
  "units": {"distance": "metres", "angle": "radians"},
  "frame": "NW room corner origin, x along the north wall",
  "landmarks": [
    {"id": 0, "x": 1.00, "y": 0.00, "yaw": 1.5708},
    {"id": 1, "x": 2.50, "y": 0.40, "yaw": 3.1416}
  ]
}
```

Space tags so 1–3 are visible from every point of the course — roughly one
tag per 1–1.5 m of track. If zero tags are visible the filter coasts blind on
its motion model.

**Aim each tag at a point on the track a couple of metres *before* it along
the driving direction, not perpendicular at the track.** A tag mounted
square-on to the track edge is seen at roughly 70° skew by an approaching
car and will not detect; angling it toward oncoming traffic keeps the
approach skew inside the detector's working range. Record whatever yaw you
actually mounted in the map.

## How to use

Put the following in `myconfig.py` and wire the parts into your vehicle:

```python
APRILTAG_MAP_PATH = "maps/track_v1.json"
APRILTAG_SIZE_M = 0.16          # measured black-square side, NOT nominal
APRILTAG_CAMERA_PARAMS = (600.0, 600.0, 320.0, 240.0)  # fx, fy, cx, cy
APRILTAG_PARTICLES = 500
```

```python
from donkeycar.parts.apriltag_localizer import (
    AprilTagCamera, AprilTagLocalizer, AprilTagMapDiff)

detector = AprilTagCamera(cfg.APRILTAG_CAMERA_PARAMS, cfg.APRILTAG_SIZE_M)
localizer = AprilTagLocalizer(detector, cfg.APRILTAG_MAP_PATH,
                              particles=cfg.APRILTAG_PARTICLES)
V.add(localizer, inputs=['cam/image_array'],
      outputs=['pos/x', 'pos/y', 'pos/theta', 'apriltag/detections'])

differ = AprilTagMapDiff(cfg.APRILTAG_MAP_PATH)
V.add(differ,
      inputs=['pos/x', 'pos/y', 'pos/theta', 'apriltag/detections'],
      outputs=['apriltag/map_events'])
```

`pos/x`, `pos/y`, `pos/theta` can feed the
[path following](path_following.md) parts in place of GPS, and
`apriltag/map_events` is a list of plain dicts ready for telemetry:

```json
{"tag_id": 7, "label": "MOVED", "evidence": 3, "x": 1.9, "y": 0.4}
```

## Map-change detection

`AprilTagMapDiff` is deliberately conservative:

- A verdict needs `evidence_frames` (default 3) consecutive frames of
  agreeing geometric evidence, so a single misdetection never fires an event.
- `MISSING` is only reported when the current pose says the tag should be
  clearly inside the camera's field of view and range (70% of the FOV, 80% of
  the range by default). Tags behind the car or at the FOV edge are never
  reported missing.
- `MOVED` requires the observed position to differ from the surveyed one by
  `moved_threshold_m` (default 0.30 m), and the particle filter's outlier
  mixture keeps a moved tag from dragging the pose estimate with it.

## Validate before driving

Before trusting ranges, hold one printed tag at a measured 1.00 m from the
camera and check the reported range. This one check validates the camera
intrinsics and the measured tag size together; a consistent scale error means
`APRILTAG_SIZE_M` does not match the printed square.

## Tuning

| Parameter | Default | Notes |
| --- | --- | --- |
| `particles` | 500 | more particles = smoother but slower |
| `range_std` | 0.15 m | raise if ranges are noisy (small/far tags) |
| `bearing_std` | 4° | raise for rolling-shutter or vibration blur |
| `outlier_probability` | 0.08 | weight of the uniform outlier mixture |
| `evidence_frames` | 3 | frames of agreement before a diff verdict |
| `moved_threshold_m` | 0.30 | displacement that counts as MOVED |

The parts and the simulator-based test suite live in
`donkeycar/parts/apriltag_localizer.py` and
`donkeycar/tests/test_apriltag_localizer.py`.
