# projectile-aim

`projectile-aim` is a TypeScript library for turning shot calculations in 3D space into aiming information.

The goal is to stay generic: external libraries provide the concrete projectile trajectory math, and this package provides the generic outline for evaluating those trajectories into aiming-oriented results.

## Project structure

- `src/` contains the TypeScript source.
- `lib/` contains compiled output.
- `test/` contains automated tests.
- `docs/` contains project notes and design docs.

## Scripts

- `pnpm build` compiles the library into `lib/`.
- `pnpm test` runs the test suite.
- `pnpm typecheck` validates the TypeScript types.
- `pnpm clean` removes build output.

## Current API

The current scaffold includes:

- `Vector3` for neutral 3D coordinates.
- `TrajectoryProvider<TContext, TCandidate>` for adapting concrete simulation libraries.
- `LaunchCandidate` and `CandidateGenerator<TContext, TCandidate>` for describing possible launches.
- `solveAim()` for scoring candidate trajectories against target geometry.
- `createDirectAimSolution()` as a simple direct-line fallback.
- `GridCandidateGenerator` and `FunctionalTrajectoryProvider` as starter utilities.
- typed trajectory provenance for piecewise simulation, formulaic solutions, and condition-based solvers.

## Intended architecture

This package is designed around a split of responsibilities:

1. A concrete library owns projectile physics and path simulation.
2. That library is adapted into a `TrajectoryProvider`.
3. `projectile-aim` evaluates candidate launches and returns the wanted yaw, pitch, and related aim info.

The solver carries your own `context` and candidate subtype all the way through, so adapters can attach engine-specific metadata without changing the core library.

Trajectory providers can also describe how a result was produced:

- `piecewise` for sampled or step-based simulation
- `formulaic` for closed-form or direct equations
- `condition` for ODE/root-finding or other solvers that target a condition

That provenance is preserved on `solution.trajectory.info` with typed fields appropriate to the computation kind.

Targets are geometry-first. A target can expose `contains(point)`, `intersects(point)`, or `isHit(point)`, plus optional helpers like `distanceTo(point)` and `referencePoint`, so aim-solving can work against AABBs or other spatial shapes instead of only point tolerances.

## Example shape

```ts
import { solveAim, type TrajectoryProvider } from "projectile-aim";

const provider: TrajectoryProvider<MyContext, MyCandidate> = {
  getTrajectory(context, candidate) {
    return {
      info: {
        kind: "piecewise",
        integrationMethod: "rk4",
      },
      samples: simulateWithMyProjectileLibrary(context, candidate),
    };
  },
};

const solution = solveAim({
  context,
  provider,
  candidates,
});
```

## Development

1. Install dependencies with `pnpm install`.
2. Build with `pnpm build`.
3. Run tests with `pnpm test`.

## Notes

This is intentionally a starting point. The structure is ready for concrete adapters once the exact trajectory libraries and launch constraints are defined.
