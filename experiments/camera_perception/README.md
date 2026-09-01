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

## Results

To be completed after running the experiment.

## Observations

The LEGO bricks were clearly visible in the depth visualization when held above the table. When placed directly on the table, the bricks became substantially less distinguishable from the table surface in the depth visualization.

## Failure Modes / Limitations

Depth-based perception may have difficulty distinguishing a LEGO brick from the supporting table surface when the brick is in direct contact with the table.

## What I Learned

To be completed.

## Next Question

Can we retrieve a meaningful depth measurement for a specific pixel in the image?
