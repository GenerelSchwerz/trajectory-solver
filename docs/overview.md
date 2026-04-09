# Projectile Aim

`projectile-aim` is intended to be a generic translation layer between raw shot calculations in 3D space and the aiming information that a caller actually needs.

## Initial direction

- Keep the public API engine-agnostic.
- Accept neutral geometric inputs such as vectors, origins, targets, candidate launch angles, and trajectory results.
- Return aiming-oriented outputs such as pitch, yaw, direction, distance-to-target, and attachable metadata.
- Leave room for future adapters that can map engine-specific projectile simulators into a shared trajectory-provider contract.
- Preserve caller-defined context and candidate types so this package does not become coupled to any one upstream projectile library.
- Preserve typed information about whether a trajectory came from piecewise simulation, a formulaic solution, or a condition-solving method.

## Abstraction

This package is centered around three generic pieces:

- `TrajectoryProvider<TContext, TCandidate, TInfo>`: adapts an external projectile simulator and yields either sampled output alone or sampled output plus typed trajectory provenance.
- `CandidateGenerator<TContext, TCandidate>` or explicit candidate lists: defines the launch angles and optional speed values to try.
- `solveAim()`: evaluates each candidate trajectory against a generic target and returns the best yaw/pitch result.

That means this library does not need to own the projectile physics itself. Instead, another package can provide the real simulation logic and this package can stay focused on scoring and selecting aim solutions.

## Trajectory provenance

Providers can describe how a trajectory was produced:

- `piecewise`: sampled simulation, integration, or stepped playback
- `formulaic`: direct equations or closed-form results
- `condition`: solver-driven outputs such as root-finding, boundary matching, or ODE-derived conditions

That typed provenance is preserved on the returned aim solution so downstream code can surface richer diagnostics when they exist.

## Planned evolution

- Add richer ballistic solver interfaces and search strategies.
- Support moving targets and time-of-flight aware aiming.
- Provide adapter helpers for concrete runtimes once those use cases are defined.
