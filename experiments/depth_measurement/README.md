# Experiment 2 — Depth Measurement

## Question

Can we retrieve a meaningful depth measurement for a specific pixel in the image?

## Motivation

The previous experiment established that the RealSense D435i provides a depth image through ROS 2.

However, visualizing the depth image is not sufficient for robotic manipulation. We need to extract a quantitative distance measurement from individual pixels.

This experiment investigates how depth values are represented and how they can be converted into a physical distance.

## Method

A ROS 2 Python node will:

- Subscribe to the depth image.
- Convert the ROS image message to an OpenCV array.
- Select a pixel (u, v).
- Retrieve its raw depth value.
- Convert the raw value into a physical distance.
- Display the measurement.

The experiment will initially use individual pixels before investigating more robust multi-pixel measurements.

## Results

TBC

## Observations

TBC

## Limitaions

TBC

## What I  Learned

TBC

## Conclusion

TBC

## Next Question

TBC
