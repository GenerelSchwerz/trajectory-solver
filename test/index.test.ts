import test from "node:test";
import assert from "node:assert/strict";

import {
  createDirectAimSolution,
  GridCandidateGenerator,
  solveAim,
  solveAimWithPitchHeuristic,
  type AimContext,
  type LaunchCandidate,
  type TrajectoryProvider,
} from "../src/index";
import {
  createCandidates,
  createContext,
  createDistanceToPoint,
  createIdealPointHeuristicContext,
  createLinearProvider,
  nearlyEqual,
  createPitchHeuristicContext,
  createPitchHeuristicProvider,
  createPointGeometry,
  createSegmentHitContext,
  createSegmentHitProvider,
} from "./testUtils";

test("createDirectAimSolution returns distance and normalized direction", () => {
  const solution = createDirectAimSolution({
    origin: { x: 0, y: 0, z: 0 },
    target: {
      referencePoint: { x: 0, y: 0, z: 10 },
      geometry: createPointGeometry({ x: 0, y: 0, z: 10 }),
    },
  });

  assert.equal(solution.distance, 10);
  assert.deepEqual(solution.direction, { x: 0, y: 0, z: 1 });
  assert.equal(solution.yawRadians, 0);
  assert.equal(solution.pitchRadians, 0);
  assert.equal(solution.trajectory.info.kind, "formulaic");
});

test("solveAim selects the candidate whose trajectory reaches the target", () => {
  const context: AimContext = createContext(
    { x: 0, y: 0, z: 0 },
    { x: 10, y: 0, z: 0 },
  );
  context.target.geometry.distanceTo = createDistanceToPoint({ x: 10, y: 0, z: 0 });

  const candidates = createCandidates(
    { yawRadians: 0, pitchRadians: 0, label: "forward-z" },
    { yawRadians: Math.PI / 2, pitchRadians: 0, label: "forward-x" },
  );
  const provider: TrajectoryProvider = createLinearProvider();

  const solution = solveAim({
    context,
    provider,
    candidates,
  });

  assert.ok(solution);
  assert.equal(solution?.hit, true);
  assert.equal(solution?.candidate.label, "forward-x");
  assert.equal(solution?.tick, 10);
  assert.equal(solution?.trajectory.info.kind, "piecewise");
});

test("solveAim preserves conditional solver metadata when provided", () => {
  const context: AimContext = {
    origin: { x: 0, y: 0, z: 0 },
    target: {
      referencePoint: { x: 0, y: 3, z: 6 },
      geometry: {
        isHit: createPointGeometry({ x: 0, y: 3, z: 6 }).contains,
      },
    },
  };

  const candidates = createCandidates(
    { yawRadians: 0, pitchRadians: 0.5, label: "root-solved" },
  );

  const provider: TrajectoryProvider<
    AimContext,
    LaunchCandidate,
    {
      kind: "condition";
      condition: string;
      solver: string;
      iterations: number;
      converged: boolean;
      rootValue: number;
    }
  > = {
    getTrajectory() {
      return {
        info: {
          kind: "condition",
          condition: "height(targetY) = 3",
          solver: "newton-raphson",
          iterations: 4,
          converged: true,
          rootValue: 0.5,
        },
        samples: [
          {
            tick: 6,
            position: { x: 0, y: 3, z: 6 },
          },
        ],
      };
    },
  };

  const solution = solveAim({
    context,
    provider,
    candidates,
  });

  assert.ok(solution);
  assert.equal(solution?.trajectory.info.kind, "condition");
  assert.equal(solution?.trajectory.info.solver, "newton-raphson");
  assert.equal(solution?.trajectory.info.converged, true);
});

test("solveAim can hit a target through segment intersection between samples", () => {
  const context = createSegmentHitContext();
  const candidates = createCandidates(
    { yawRadians: 0, pitchRadians: 0, label: "miss-z" },
    { yawRadians: Math.PI / 2, pitchRadians: 0, label: "segment-hit-x" },
  );
  const provider: TrajectoryProvider = createSegmentHitProvider();

  const solution = solveAim({
    context,
    provider,
    candidates,
  });

  assert.ok(solution);
  assert.equal(solution?.hit, true);
  assert.equal(solution?.candidate.label, "segment-hit-x");
  assert.deepEqual(solution?.closestPosition, { x: 5, y: 0, z: 0 });
});

test("GridCandidateGenerator supports fixed yaw and yaw search", () => {
  const context = createContext(
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 10 },
  );

  const fixedYawGenerator = new GridCandidateGenerator({
    fixedYaw: Math.PI / 2,
    pitch: { start: 0, end: 0, step: 0.01 },
  });
  const searchedYawGenerator = new GridCandidateGenerator({
    yawSearch: { start: -0.1, end: 0.1, step: 0.1 },
    pitch: { start: 0, end: 0, step: 0.01 },
    yawMode: "relative",
    provideYaw: () => 1,
  });
  const providedYawGenerator = new GridCandidateGenerator({
    pitch: { start: 0, end: 0, step: 0.01 },
    yawMode: "relative",
    provideYaw: () => 1.5,
  });

  const fixedYawCandidates = Array.from(fixedYawGenerator.getCandidates(context));
  const searchedYawCandidates = Array.from(searchedYawGenerator.getCandidates(context));
  const providedYawCandidates = Array.from(providedYawGenerator.getCandidates(context));

  assert.equal(fixedYawCandidates.length, 1);
  assert.equal(fixedYawCandidates[0]?.yawRadians, Math.PI / 2);
  assert.equal(providedYawCandidates.length, 1);
  assert.equal(providedYawCandidates[0]?.yawRadians, 1.5);
  assert.deepEqual(
    searchedYawCandidates.map((candidate) => candidate.yawRadians),
    [0.9, 1, 1.1],
  );
});

test("solveAimWithPitchHeuristic homes in after overshooting", () => {
  const context = createPitchHeuristicContext();
  const provider = createPitchHeuristicProvider();
  const solution = solveAimWithPitchHeuristic({
    context,
    provider,
    candidates: createCandidates({
      yawRadians: Math.PI / 2,
      pitchRadians: 0,
      label: "seed",
    }),
    initialDeltaRadians: 0.2,
    maxIterations: 25,
  });

  assert.ok(solution);
  assert.equal(solution?.hit, true);
  assert.equal(solution?.pitchRadians, 0.4);
  assert.deepEqual(solution?.closestPosition, { x: 10, y: 4, z: 0 });
  // assert.deepEqual(
  //   (solution?.candidate.metadata?.pitchHeuristic as { history: { deltaRadians: number }[] }).history.map(
  //     (entry) => entry.deltaRadians,
  //   ),
  //   [0, 0.2, 0.4, 0.2],
  // );
});

test("solveAimWithPitchHeuristic returns the midpoint of the hittable pitch band", () => {
  const context = createIdealPointHeuristicContext();
  const provider = createPitchHeuristicProvider();
  const solution = solveAimWithPitchHeuristic({
    context,
    provider,
    candidates: createCandidates({
      yawRadians: Math.PI / 2,
      pitchRadians: 0.36,
      label: "ideal-seed",
    }),
    initialDeltaRadians: 0.04,
    maxIterations: 10,
  });

  assert.ok(solution);
  assert.equal(solution?.hit, true);
  assert.equal(nearlyEqual(solution?.pitchRadians ?? 0, 0.4), true);
  assert.equal(nearlyEqual(solution?.closestPosition.x ?? 0, 10), true);
  assert.equal(nearlyEqual(solution?.closestPosition.y ?? 0, 4.0), true);
  assert.equal(nearlyEqual(solution?.closestPosition.z ?? 0, 0), true);
});
