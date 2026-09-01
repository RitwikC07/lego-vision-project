# Experiment 2 — Depth Measurement

## Question

Can we retrieve a meaningful depth measurement for a specific pixel in the image?

## Motivation

The previous experiment established that the RealSense D435i provides a depth image through ROS 2.

Here we extract a quantitative distance measurement from individual pixels and determine how reliable those measurements are.

This experiment investigates how depth values behave, how stable they are, how the table surface can be modelled, and how an object can be separated from the table using depth.

## Sub-Experiments

We have used a book kept on the table as our experimental region.

* Subscribe to the depth image.
* Convert the ROS image message to an OpenCV array.
* Select individual pixels and regions of interest.
* Measure depth over multiple frames.
* Evaluate depth stability and invalid measurements.
* Investigate depth discontinuities and edges.
* Estimate the table surface as a depth plane.
* Calculate depth residuals relative to the table plane.
* Detect the book using its negative depth residual.
* Estimate the book's image-space bounding box and pixel centroid.
* Test detection while moving the book laterally and towards the image boundaries.

The depth image used during the experiments was **848 × 480** pixels.

## Results

### Raw depth stability

Measurements from individual pixels were stable when the camera was positioned approximately perpendicular to the book/table surface.

Example measurements:

| Approx. physical distance | Measured depth |
| ------------------------: | -------------: |
|                     32 cm |            319 |
|                     35 cm |            347 |
|                     38 cm |            378 |

The measurements changed consistently with physical distance, although the raw depth scale was not formally established during this experiment.

### Table plane

The table was not at exactly the same depth across the image. A plane was fitted using measurements of the table:

```text
Z = -0.01136354x - 0.00771668y + 424.66
```

At the centre of the image, the expected table depth was approximately:

```text
417.99
```

The table showed approximately **10 units of horizontal depth variation** across the analysed region, demonstrating that a single global depth threshold would not be reliable.

After subtracting the fitted table plane, the table residual had a standard deviation of approximately:

```text
0.86
```

This showed that plane correction significantly reduced the apparent depth variation caused by the table's orientation.

### Book detection

The book was detected using the residual:

```text
residual = measured_depth - expected_table_depth
```

Because the book is closer to the camera than the table, its residual is negative.

A threshold of:

```text
residual < -20
```

was used to identify object pixels.

For well-centred measurements, the book residual was approximately:

```text
-44
```

while the table residual was approximately:

```text
0 ± 1
```

This provided a strong separation between the book and the table.

### Lateral localization

Moving the book from left to right produced a corresponding movement in the detected pixel centroid:

| Position       | Object centroid X |
| -------------- | ----------------: |
| Further left   |            263.70 |
| Slightly left  |            330.49 |
| Centre         |            398.28 |
| Slightly right |            452.74 |
| Further right  |            545.26 |

The detected centroid moved monotonically with the physical movement of the book.

This demonstrates that the depth-based detector can provide useful **image-space localization**.

### Boundary robustness

The detector was also tested with the book progressively approaching and leaving the image boundaries.

Representative object detection ratios were:

| Position                | Object pixels |
| ----------------------- | ------------: |
| Left inside             |        28.86% |
| Left near boundary      |         9.44% |
| Left outside            |         0.00% |
| Centre                  |        31.14% |
| Right inside            |        31.00% |
| Right near boundary     |        21.95% |
| Right partially outside |         9.27% |
| Centre again            |        31.47% |

When the book was completely outside the tested region, **no object pixels were detected**.

The table-only measurement in that case had a minimum residual of approximately:

```text
-18.67
```

which remained above the `-20` detection threshold.

This provided a useful false-positive check for the chosen threshold.

## Observations

* Individual depth measurements are reasonably stable when the camera and target surface are positioned consistently.
* Depth across the table is not constant because of the camera/table geometry.
* A single global depth threshold is therefore insufficient.
* Fitting a plane to the table provides a much better reference surface.
* Depth residuals provide a strong separation between the book and the table.
* The book produced residuals around `-44` relative to the table.
* The `-20` residual threshold successfully separated the book from the table in the tested setup.
* Simple centre-versus-surrounding depth discontinuities were not reliable enough to use as the primary detector.
* Depth-edge detection was also too sparse and inconsistent to be used as the primary detector.
* Residual-based detection remained effective when the book moved away from the centre of the image.
* The detected pixel centroid followed the lateral movement of the book.
* Detection decreased naturally as the book approached the image boundaries.
* No object was detected when the book was completely outside the tested region.

## Limitations

* The depth scale was not formally established from the camera configuration during this experiment.
* The `-20` residual threshold was validated for the tested book/table setup and should not yet be considered a universal threshold.
* The table plane was measured for this particular physical setup. Changing the camera position or orientation would require recalculating the plane.
* The bounding box was partially clipped by the analysis-region boundaries in several experiments, so its dimensions should not yet be interpreted as physical object dimensions.
* The detected centroid is currently an image-space `(u, v)` position, not a calibrated 3D position.
* The experiments primarily considered a relatively flat object on a flat table.
* More complex objects, surfaces, lighting conditions, and camera poses have not yet been tested.

## What I Learned

The important result of this experiment is that **raw depth alone is not the best representation for object detection**.

The table itself contains a depth gradient, so comparing an object's depth against one fixed value is unreliable.

Instead, the table can be represented as a plane and each measured depth can be compared against the expected depth of that plane.

This gives a residual:

```text
measured depth - expected table depth
```

For the tested setup:

* Table → residual close to `0`
* Book → residual around `-44`

This creates a much cleaner representation for detecting objects above the table.

I also learned that depth can provide more than just a distance measurement. Once the object pixels are identified, their spatial distribution can be used to estimate the object's position in the image.

## Conclusion

The RealSense depth image provides sufficiently stable measurements for the tested setup.

More importantly, modelling the table as a depth plane and using depth residuals provides a robust way to distinguish the book from the table.

The final detector was able to:

* Estimate the expected table depth.
* Detect the book using depth residuals.
* Localize the detected book in image space.
* Follow the book during lateral movement.
* Handle partial visibility near image boundaries.
* Avoid detecting the book when it was outside the tested region.

The experiment therefore answered the original question:

> **Yes, we can retrieve meaningful depth information from the depth image, and that information can be used to detect and localize an object relative to a known surface.**

However, the result is currently expressed in **image-space coordinates and depth residuals**. It is not yet a calibrated 3D position.

## Next Question

**Can we convert the detected pixel position and depth measurement into a meaningful 3D position relative to the camera?**

