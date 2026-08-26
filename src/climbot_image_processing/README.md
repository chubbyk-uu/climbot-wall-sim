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
  --input-run /home/jerry/climbot_data/example/r000001_20260826T120000Z_x \
  --output-dir /home/jerry/climbot_processed/example_run \
  --flat-field-file /home/jerry/climbot_calibration/flat_field.npz \
  --dark-frame /home/jerry/climbot_calibration/dark.png \
  --denoise median3
```

`--output-dir` must be an absolute, nonexistent directory outside the source
archive. The output has `images/`, copied-and-extended `metadata/`, the frozen
mount snapshot, a rectified camera calibration, and `processing_manifest.json`. The manifest
contains source/output SHA-256 links and processing parameters but deliberately
does not store machine-local absolute input paths. By default only archives
sealed as `completed` are accepted; `--allow-incomplete` is solely for an
explicit forensic run.
