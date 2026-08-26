# Climbot Mosaic

`climbot_mosaic` is the offline successor to `climbot_image_processing`. It
never reads a ROS graph or Gazebo truth while building a mosaic: its only image
inputs are completed, independently verified processed archives.

The first delivered command validates one or more processed archives before
expensive feature extraction begins:

```bash
ros2 run climbot_mosaic validate_mosaic_inputs \
  --input-run /home/jerry/climbot_data/processed-inspection-dataset-horizontal-025mm-20260826 \
  --input-run /home/jerry/climbot_data/processed-inspection-dataset-vertical-025mm-20260826
```

It verifies every processed PNG SHA-256, every processing label, rectified
camera calibration, frozen camera mount, finite exposure pose and covariance.
For the first planar-wall implementation all input runs must share one
rectified camera model and mount snapshot. The JSON result is a stable,
machine-readable preflight summary; it does not write output directories.

Later stages add wall-plane projection, spatial overlap candidates, feature
matching, robust global optimization and tiled BigTIFF fusion. See
[`docs/MOSAIC_PLAN.md`](../../docs/MOSAIC_PLAN.md) for the staged design and
baseline-first acceptance policy.
