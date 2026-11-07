# Third-party sources

The raceline optimizer uses two Waterloo repositories. (not
committed here). Clone them into this directory
(they are gitignored) with the ommand below.

- raceline_optimization: orchestration, track preparation and future trajectory stages.
- trajectory_planning_helpers: the spline, quadratic-program and resampling implementation.

## Setup (first time only)

Run from the `racerbot_a` repository root:

```bash
git clone https://github.com/CL2-UWaterloo/Raceline-Optimization third_party/raceline_optimization
git -C third_party/raceline_optimization checkout 9290c5d503462e46f7e3e9033002e7ddf165ba7b

git clone https://github.com/CL2-UWaterloo/trajectory_planning_helpers third_party/trajectory_planning_helpers
git -C third_party/trajectory_planning_helpers checkout fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56
git -C third_party/trajectory_planning_helpers apply ../patches/scipy-flat-vector.patch
```
