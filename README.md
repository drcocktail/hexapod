# Hexapod Motion Planning

MATLAB simulator for a hexapod crossing procedural voxel terrain with volumetric body-clearance checks, nominal leg reachability checks, RRT path planning, and 3D animation.

## Structure

```text
main.m
src/+motionplanning/
  runSimulation.m
  +config/defaultConfig.m
  +environment/
  +planning/
  +robot/
  +utils/
  +visualization/
tests/testMotionPlanning.m
```

The root `main.m` is only a launcher. The simulator code lives in the `motionplanning` package:

- `+environment`: terrain generation, flattening, height queries, local grid windows, domain checks.
- `+robot`: hexapod dimensions, chassis geometry, terrain-following pose estimates, foot targets, leg IK, tripod stability checks.
- `+planning`: RRT sampling, steering, spatial nearest-node search, state validation, edge validation, path reconstruction.
- `+visualization`: terrain/path rendering and robot animation.
- `+config`: default simulation parameters.

## Run

From MATLAB at the repository root:

```matlab
main
```

Run with a fixed seed:

```matlab
main('Seed', 42)
```

Run without visualization:

```matlab
result = main('Seed', 42, 'Render', false);
```

## Tests

From MATLAB at the repository root:

```matlab
addpath('src')
results = runtests('tests');
table(results)
```

## Notes

- The planner is named `planRRT` because it is RRT, not RRT*. It does not currently do cost optimization or rewiring.
- State validity checks body clearance using chassis center height: `max terrain under body + belly clearance + bodyHeight / 2`.
- State validity estimates terrain-following pitch/roll during planning and rejects poses above configured orientation limits.
- Static stability requires the center of mass projection to remain inside both alternating tripod support polygons with a configurable margin.
- Edge validity samples intermediate states, so a path cannot pass through invalid terrain just because its endpoints are valid.
- Terrain uses interpolated octave value noise instead of Gaussian convolution, so lowering frequency does not create a huge smoothing kernel.
- RRT nearest-neighbor lookup uses an exact spatial bucket index by default. The original brute-force search remains available by setting `cfg.planning.nearestMode` to another value.
- This is still a 2-D state-space planner with planning-time pose validation, not a full SE(3) kinodynamic planner, and it is still RRT rather than RRT*.
