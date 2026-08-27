# Climbot Image Processing

`climbot_image_processing` is the offline successor to the G4 raw archive.
It has no ROS graph, Gazebo or controller dependency: it verifies one sealed
archive, writes a new directory, and never modifies raw images or labels.

The first verified chain is fixed in this order:

1. Verify the completed archive manifest, every PNG SHA-256, label and camera calibration.
2. Optionally subtract a mono8 dark frame and apply a checked NPZ flat-field gain in distorted sensor pixels.
3. Optionally apply `median3` denoising.
4. Rectify the result with the archived `plumb_bob` `K/D` camera calibration.

Run it after sourcing the workspace:

```bash
ros2 run climbot_image_processing process_inspection_archive \
  --input-run "$CLIMBOT_DATA_ROOT/example/r000001_20260826T120000Z_x" \
  --output-dir "$CLIMBOT_DATA_ROOT/processed/example_run" \
  --flat-field-file "$CLIMBOT_DATA_ROOT/calibration/flat_field.npz" \
  --dark-frame "$CLIMBOT_DATA_ROOT/calibration/dark.png" \
  --denoise median3 \
  --jobs auto --memory-budget-gb 4.0
```

`--output-dir` must be an absolute, nonexistent directory outside the source
archive. The output has `images/`, copied-and-extended `metadata/`, the frozen
mount snapshot, a rectified camera calibration, and `processing_manifest.json`. The manifest
contains source/output SHA-256 links and processing parameters but deliberately
does not store machine-local absolute input paths. By default only archives
sealed as `completed` are accepted; `--allow-incomplete` is solely for an
explicit forensic run.

`--jobs` is `auto` or a positive process count. `auto` chooses no more than
eight workers after applying `--memory-budget-gb` (default `4.0`). The camera
remap is built once and installed read-only in each worker; each worker limits
OpenCV to one internal thread so process parallelism does not oversubscribe the
machine. The resolved worker count and frame-processing duration are recorded
in `processing_manifest.json` under `execution`.
