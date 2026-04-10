import assert from "node:assert/strict";
import { test } from "node:test";
import { performance } from "node:perf_hooks";

import {
  AABBUtils,
  createReplayBot,
  Enderman,
  toVector3,
  Vec3,
  type Bot,
} from "./setup";
import {
  createCandidateGenerator,
  createReplayContext,
  estimatePitchWithLinearDrag,
  REPLAY_TEST_DV_STEP,
  REPLAY_TEST_MAX_TICKS,
  solveReplayAimWithCollisionFallbackPitchHeuristic,
  solveReplayAimWithCollisionPitchHeuristic,
  solveReplayAimWithPitchHeuristic,
} from "./enderTestUtils";

function createReplayFixture(
  landingPos: Vec3,
  options?: {
    bot?: Bot;
    maxTicks?: number;
    dvStep?: number;
  },
) {
  const bot = options?.bot ?? createReplayBot();
  const enderman = new Enderman(bot);
  const landingAABB = AABBUtils.getEntityAABBRaw({
    position: landingPos,
    height: 1.8,
    width: 0.6,
  });
  const replayContext = createReplayContext(
    toVector3(bot.entity.position),
    landingAABB,
    toVector3(landingPos),
  );

  enderman.maxTicks = options?.maxTicks ?? REPLAY_TEST_MAX_TICKS;
  enderman.dvStep = options?.dvStep ?? REPLAY_TEST_DV_STEP;

  return {
    bot,
    enderman,
    landingPos,
    landingAABB,
    replayContext,
  };
}

function getOriginVelocity(bot: Bot) {
  return bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
}

function measureAverageMs<TSolution extends { hit: boolean } | null>(
  iterations: number,
  run: () => { elapsedMs: number; solution: TSolution },
) {
  let totalElapsedMs = 0;
  let lastResult: ReturnType<typeof run> | null = null;

  for (let index = 0; index < iterations; index += 1) {
    lastResult = run();
    totalElapsedMs += lastResult.elapsedMs;
  }

  return {
    averageElapsedMs: totalElapsedMs / iterations,
    lastResult,
  };
}

function measureEnderAverageMs<TResult>(
  iterations: number,
  run: () => TResult,
) {
  let totalElapsedMs = 0;
  let lastResult: TResult | null = null;

  for (let index = 0; index < iterations; index += 1) {
    const startedAt = performance.now();
    lastResult = run();
    totalElapsedMs += performance.now() - startedAt;
  }

  return {
    averageElapsedMs: totalElapsedMs / iterations,
    lastResult,
  };
}

function printBenchmark(input: {
  label: string;
  iterations: number;
  enderMs: number;
  noCollisionNoVerificationMs?: number;
  noCollisionCheckMs: number;
  collisionMs: number;
}) {
  console.log(
    [
      `[bench:${input.label}]`,
      `iterations=${input.iterations}`,
      `ender=${input.enderMs.toFixed(3)}ms`,
      `no_collision_no_verification=${input.noCollisionNoVerificationMs?.toFixed(3) ?? "N/A"}ms`,
      `no_collision_check=${input.noCollisionCheckMs.toFixed(3)}ms`,
      `collision=${input.collisionMs.toFixed(3)}ms`,
      `no_collision_no_verification_vs_ender=${input.noCollisionNoVerificationMs ? (input.noCollisionNoVerificationMs / Math.max(input.enderMs, 1e-9)).toFixed(2) : "N/A"}x`,
      `no_collision_check_vs_ender=${(input.noCollisionCheckMs / Math.max(input.enderMs, 1e-9)).toFixed(2)}x`,
      `collision_vs_ender=${(input.collisionMs / Math.max(input.enderMs, 1e-9)).toFixed(2)}x`,
      `collision_vs_no_collision_check=${(input.collisionMs / Math.max(input.noCollisionCheckMs, 1e-9)).toFixed(2)}x`,
    ].join(" "),
  );
}

test("benchmark: replay seed compares ender vs heuristic variants", () => {
  const options = { maxTicks: REPLAY_TEST_MAX_TICKS, dvStep: REPLAY_TEST_DV_STEP };
  const fixture = createReplayFixture(new Vec3(-79.5, 4.0, -29.5), options);
  const originVelocity = getOriginVelocity(fixture.bot);
  const baseline = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
  )!;
  const iterations = 100;

  assert.ok(baseline.hit, "benchmark replay baseline should produce a hit");

  const pitchSeed = estimatePitchWithLinearDrag(
    fixture.bot,
    baseline.yaw,
    toVector3(fixture.landingPos),
    originVelocity,
  );

  const initialCandidates = Array.from(
    createCandidateGenerator(
      pitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
  );

  const enderSolver = measureEnderAverageMs(iterations, () =>
    fixture.enderman.shotToAABB(
      fixture.landingAABB,
      fixture.landingPos,
    ));
  const noCollisionNoVerificationSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      false,
      fixture.enderman.maxTicks,
    ));
  const noCollisionCheckSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      undefined,
      fixture.enderman.maxTicks,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      undefined,
      fixture.enderman.maxTicks,
    ));

  printBenchmark({
    label: "replay-seed",
    iterations,
    enderMs: enderSolver.averageElapsedMs,
    noCollisionNoVerificationMs: noCollisionNoVerificationSolver.averageElapsedMs,
    noCollisionCheckMs: noCollisionCheckSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.ok(enderSolver.lastResult?.hit, "ender replay benchmark should return a hit");
  assert.equal(noCollisionNoVerificationSolver.lastResult?.solution?.hit, true);
  assert.equal(noCollisionCheckSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});

test("benchmark: high-arc minFlightTicks compares ender vs heuristic variants", () => {
  const fixture = createReplayFixture(new Vec3(-121.0, 4.0, -29.5), {
    maxTicks: REPLAY_TEST_MAX_TICKS,
  });
  const originVelocity = getOriginVelocity(fixture.bot);
  const minFlightTicks = 30;
  const baseline = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
    undefined,
    minFlightTicks,
  )!;
  const iterations = 100;

  assert.ok(baseline.hit, "benchmark high-arc baseline should produce a hit");
  assert.ok(
    baseline.ticks >= minFlightTicks,
    "benchmark high-arc baseline should respect minFlightTicks",
  );

  const pitchSeed = estimatePitchWithLinearDrag(
    fixture.bot,
    baseline.yaw,
    toVector3(fixture.landingPos),
    originVelocity,
  );

  const initialCandidates = Array.from(
    createCandidateGenerator(
      pitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
  );

  const enderSolver = measureEnderAverageMs(iterations, () =>
    fixture.enderman.shotToAABB(
      fixture.landingAABB,
      fixture.landingPos,
      undefined,
      minFlightTicks,
    ));
  const noCollisionNoVerificationSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      false,
      fixture.enderman.maxTicks,
    ));
  const noCollisionCheckSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      undefined,
      fixture.enderman.maxTicks,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
      undefined,
      undefined,
      fixture.enderman.maxTicks,
    ));

  printBenchmark({
    label: "high-arc-seed",
    iterations,
    enderMs: enderSolver.averageElapsedMs,
    noCollisionNoVerificationMs: noCollisionNoVerificationSolver.averageElapsedMs,
    noCollisionCheckMs: noCollisionCheckSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.ok(enderSolver.lastResult?.hit, "ender high-arc benchmark should return a hit");
  assert.equal(noCollisionNoVerificationSolver.lastResult?.solution?.hit, true);
  assert.equal(noCollisionCheckSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});

test("benchmark: blocked alternate-arc compares ender vs checked variants", () => {
  const landingPos = new Vec3(-79.0, 4.0, -39.5);
  const blockingBlocks = [new Vec3(-77, 4, -36), new Vec3(-77, 5, -36)];
  const openFixture = createReplayFixture(landingPos);
  const openBaseline = openFixture.enderman.shotToAABB(
    openFixture.landingAABB,
    openFixture.landingPos,
  )!;
  const openArc = openFixture.enderman.shotToAABB(
    openFixture.landingAABB,
    openFixture.landingPos,
    undefined,
    20,
  )!;
  const blockedFixture = createReplayFixture(landingPos, {
    bot: createReplayBot({ solidBlocks: blockingBlocks }),
  });
  const blockedOriginVelocity = getOriginVelocity(blockedFixture.bot);
  const iterations = 100;

  assert.ok(openBaseline.hit, "benchmark blocked-case baseline should produce a hit");
  assert.ok(openArc.hit, "benchmark blocked-case alternate arc should produce a hit");

  const initialCandidates = Array.from(
    createCandidateGenerator(
      openBaseline.pitch,
      { provideYaw: () => openBaseline.yaw },
    ).getCandidates(blockedFixture.replayContext),
  );
  const alternateCandidates = Array.from(
    createCandidateGenerator(
      openArc.pitch,
      { provideYaw: () => openArc.yaw },
    ).getCandidates(blockedFixture.replayContext),
  );
  const enderSolver = measureEnderAverageMs(iterations, () =>
    blockedFixture.enderman.shotToAABB(
      blockedFixture.landingAABB,
      blockedFixture.landingPos,
      undefined,
      20,
    ));
  const noCollisionCheckSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      blockedFixture.bot,
      blockedFixture.landingAABB,
      blockedFixture.landingPos,
      blockedOriginVelocity,
      initialCandidates,
      alternateCandidates,
      undefined,
      blockedFixture.enderman.maxTicks,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      blockedFixture.bot,
      blockedFixture.landingAABB,
      blockedFixture.landingPos,
      blockedOriginVelocity,
      initialCandidates,
      alternateCandidates,
      undefined,
      blockedFixture.enderman.maxTicks,
    ));

  printBenchmark({
    label: "blocked-alt-arc",
    iterations,
    enderMs: enderSolver.averageElapsedMs,
    noCollisionCheckMs: noCollisionCheckSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.ok(enderSolver.lastResult?.hit, "ender blocked benchmark should return a hit");
  assert.equal(noCollisionCheckSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});
