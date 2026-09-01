# Experiment 01 — Camera Perception

## Question

Can we acquire and visualize RGB and depth data from an Intel RealSense D435i through ROS 2?

## Motivation

Before attempting LEGO detection or robot manipulation, we need to establish a reliable source of visual and depth information.

The RealSense D435i provides both RGB and depth measurements, which will later allow detected LEGO objects to be localized in 3D.

## Method

A ROS 2 Python node subscribes to the RGB and depth image topics.

The RGB image is converted to an OpenCV BGR image using cv_bridge.

The depth image is received without converting its underlying numerical representation. A normalized visualization is then generated for display.

Objects (LEGO bricks) are kept in varying positions and orientations and are then observed.

## Results

The RealSense D435i was successfully accessed through ROS 2 and both RGB and depth streams were received.

### RGB Stream

- Resolution: 1280 × 720
- Encoding: `rgb8`
- Observed rate: approximately 30 Hz
- Frame: `camera_color_optical_frame`

### Depth Stream

- Resolution: 848 × 480
- Encoding: `16UC1`
- Observed rate: approximately 30 Hz
- Frame: `camera_depth_optical_frame`

### Depth Camera Intrinsics

The depth camera reported:

- `fx = 424.16`
- `fy = 424.16`
- `cx = 420.16`
- `cy = 238.57`

These parameters will be required later for converting image coordinates and depth measurements into 3D coordinates.

## Observations

The LEGO bricks were clearly visible in the depth visualization when held above the table. When placed directly on the table, the bricks became substantially less distinguishable from the table surface in the depth visualization.

This suggests that depth-based object separation may be more difficult when the object and supporting surface are at similar depths.

RGB and depth streams were successfully visualized using a Python ROS 2 node and OpenCV.

## Failure Modes / Limitations

Depth-based perception may have difficulty distinguishing a LEGO brick from the supporting table surface when the brick is in direct contact with the table.

RGB and depth images currently have different resolutions.

## What I Learned

The RealSense provides both RGB and depth information through ROS 2. RGB and depth are separate image streams and cannot automatically be treated as if their pixel coordinates correspond directly.

Depth measurements will require understanding the image encoding, depth scale, camera intrinsics, and eventually RGB-depth alignment.

## Next Question

Can we retrieve a meaningful depth measurement for a specific pixel in the image?
