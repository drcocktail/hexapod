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

- `+environment`: terrain generation, flattening, height queries, domain checks.
- `+robot`: hexapod dimensions, chassis geometry, foot targets, leg IK, reachability checks.
- `+planning`: RRT sampling, steering, nearest-node search, state validation, edge validation, path reconstruction.
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
- Edge validity samples intermediate states, so a path cannot pass through invalid terrain just because its endpoints are valid.
- Terrain smoothing uses base MATLAB `conv2`; it does not require `imgaussfilt` or Image Processing Toolbox.
