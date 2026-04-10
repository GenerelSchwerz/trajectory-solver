import assert from "node:assert/strict";
import { test } from "node:test";

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

  enderman.maxTicks = options?.maxTicks ?? 100;
  enderman.dvStep = options?.dvStep ?? 360;

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

function printBenchmark(input: {
  label: string;
  iterations: number;
  oldMs?: number;
  fallbackMs: number;
  collisionMs: number;
}) {
  console.log(
    [
      `[bench:${input.label}]`,
      `iterations=${input.iterations}`,
      `old=${input.oldMs?.toFixed(3) ?? "N/A"}ms`,
      `fallback=${input.fallbackMs.toFixed(3)}ms`,
      `collision=${input.collisionMs.toFixed(3)}ms`,
      `fallback_vs_old=${input.oldMs ? (input.fallbackMs / Math.max(input.oldMs, 1e-9)).toFixed(2) : "N/A"}x`,
      `collision_vs_old=${input.oldMs ? (input.collisionMs / Math.max(input.oldMs, 1e-9)).toFixed(2) : "N/A"}x`,
      `collision_vs_fallback=${input.fallbackMs ? (input.collisionMs / Math.max(input.fallbackMs, 1e-9)).toFixed(2) : "N/A"}x`,
    ].join(" "),
  );
}

test("benchmark: replay seed compares old vs fallback vs collision-aware", () => {
  const fixture = createReplayFixture(new Vec3(-79.5, 4.0, -29.5));
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

  const oldSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));
  const fallbackSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));

  printBenchmark({
    label: "replay-seed",
    iterations,
    oldMs: oldSolver.averageElapsedMs,
    fallbackMs: fallbackSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.equal(oldSolver.lastResult?.solution?.hit, true);
  assert.equal(fallbackSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});

test("benchmark: high-arc minFlightTicks compares old vs fallback vs collision-aware", () => {
  const fixture = createReplayFixture(new Vec3(-121.0, 4.0, -29.5), {
    maxTicks: 140,
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

  const oldSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));
  const fallbackSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      fixture.bot,
      fixture.landingAABB,
      fixture.landingPos,
      originVelocity,
      initialCandidates,
    ));

  printBenchmark({
    label: "high-arc-seed",
    iterations,
    oldMs: oldSolver.averageElapsedMs,
    fallbackMs: fallbackSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.equal(oldSolver.lastResult?.solution?.hit, true);
  assert.equal(fallbackSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});

test("benchmark: blocked alternate-arc compares old vs fallback vs collision-aware", () => {
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
  const combinedCandidates = [...initialCandidates, ...alternateCandidates];


  const fallbackSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionFallbackPitchHeuristic(
      blockedFixture.bot,
      blockedFixture.landingAABB,
      blockedFixture.landingPos,
      blockedOriginVelocity,
      initialCandidates,
      alternateCandidates,
    ));
  const collisionSolver = measureAverageMs(iterations, () =>
    solveReplayAimWithCollisionPitchHeuristic(
      blockedFixture.bot,
      blockedFixture.landingAABB,
      blockedFixture.landingPos,
      blockedOriginVelocity,
      initialCandidates,
      alternateCandidates,
    ));

  printBenchmark({
    label: "blocked-alt-arc",
    iterations,
    fallbackMs: fallbackSolver.averageElapsedMs,
    collisionMs: collisionSolver.averageElapsedMs,
  });

  assert.equal(fallbackSolver.lastResult?.solution?.hit, true);
  assert.equal(collisionSolver.lastResult?.solution?.hit, true);
});
