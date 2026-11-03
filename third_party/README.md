# Waterloo source snapshots

These two directories are complete source snapshots of the repositories and
commits recorded in UPSTREAM.json, fetched through GitHub's archive API.
The helper has one documented SciPy shape-compatibility patch (see patches/).
No numerical algorithm has been changed. Their original LGPL licenses and
attribution remain in each directory. Racerbot's MIT license does not relicense
these sources. COLCON_IGNORE excludes this directory from ROS package discovery.

- raceline_optimization: orchestration, track preparation and future trajectory stages.
- trajectory_planning_helpers: the spline, quadratic-program and resampling implementation.

Install the helper snapshot with the commands in ../tools/raceline_optimizer/README.md.
Do not install the historical raceline_optimization/requirements.txt into ROS.
