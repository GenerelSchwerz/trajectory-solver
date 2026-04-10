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
  createShotDiagnostics,
  estimatePitchWithLinearDrag,
  getClosestPointDistance,
  logReplayValues,
  REPLAY_TEST_DV_STEP,
  REPLAY_TEST_MAX_TICKS,
  solveReplayAimWithCollisionFallbackPitchHeuristic,
} from "./enderTestUtils";

const SHOW_REPLAY_VALUES_ON_FAILURE =
  process.env.SHOW_REPLAY_VALUES_ON_FAILURE !== "false";

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

function getHeuristicIterations(solution: { candidate?: { metadata?: Record<string, unknown> } } | null | undefined) {
  return (
    (
      solution?.candidate?.metadata?.pitchHeuristic as
        | { history?: unknown[] }
        | undefined
    )?.history?.length ?? 0
  );
}

function getCollisionFallbackIterations(
  solution: { candidate?: { metadata?: Record<string, unknown> } } | null | undefined,
) {
  return (
    (solution?.candidate?.metadata?.collisionFallbackPitchHeuristic as
      | {
          collisionFreeIterations?: number;
          collisionAwareEvaluations?: number;
          attemptedCandidates?: number;
          stages?: Array<{
            kind: "initial" | "alternate";
            seedPitchRadians: number;
            collisionFreeIterations: number;
            collisionFreeHit: boolean;
            collisionAwareHit: boolean;
          }>;
        }
      | undefined) ?? {
      collisionFreeIterations: 0,
      collisionAwareEvaluations: 0,
      attemptedCandidates: 0,
      stages: [],
    }
  );
}

function printReplayMetrics(input: {
  label: string;
  baselineElapsedMs: number;
  solveElapsedMs: number;
  iterations?: number;
  collisionFreeIterations?: number;
  collisionAwareEvaluations?: number;
}) {
  const parts = [
    `[new:${input.label}] baseline=${input.baselineElapsedMs.toFixed(3)}ms`,
    `solve=${input.solveElapsedMs.toFixed(3)}ms`,
  ];

  if (input.iterations !== undefined) {
    parts.push(`iterations=${input.iterations}`);
  }

  if (input.collisionFreeIterations !== undefined) {
    parts.push(`cf_iters=${input.collisionFreeIterations}`);
  }

  if (input.collisionAwareEvaluations !== undefined) {
    parts.push(`ca_checks=${input.collisionAwareEvaluations}`);
  }

  console.log(parts.join(" "));
}

function runWithFailureLogging(
  printDetails: () => void,
  assertions: () => void,
) {
  try {
    assertions();
  } catch (error) {
    printDetails();
    throw error;
  }
}

test("new solver: replay case can be evaluated through projectile-aim", () => {
  const fixture = createReplayFixture(new Vec3(-79.5, 4.0, -29.5));
  const baselineStartedAt = performance.now();
  const baseline = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
  )!;
  const baselineElapsedMs = performance.now() - baselineStartedAt;
  assert.ok(baseline?.hit, "mineflayer-ender baseline replay should produce a hit");

  const originVelocity = getOriginVelocity(fixture.bot);
  const providedYawPitchSeed = estimatePitchWithLinearDrag(
    fixture.bot,
    baseline.yaw,
    toVector3(fixture.landingPos),
    originVelocity,
  );
  const providedYawCheck = solveReplayAimWithCollisionFallbackPitchHeuristic(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
    undefined,
    undefined,
    fixture.enderman.maxTicks,
  );
  const baselineDiagnostics = createShotDiagnostics(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    baseline.yaw,
    baseline.pitch,
    originVelocity,
  );

  printReplayMetrics({
    label: "replay",
    baselineElapsedMs,
    solveElapsedMs: providedYawCheck.elapsedMs,
    iterations: getHeuristicIterations(providedYawCheck.solution),
    collisionFreeIterations: getCollisionFallbackIterations(providedYawCheck.solution).collisionFreeIterations,
    collisionAwareEvaluations: getCollisionFallbackIterations(providedYawCheck.solution).collisionAwareEvaluations,
  });

  const printDetails = logReplayValues({
    enabled: false,
    onlyOnFailure: SHOW_REPLAY_VALUES_ON_FAILURE,
    baselineElapsedMs,
    providedYawElapsedMs: providedYawCheck.elapsedMs,
    summary: {
      originVelocity: toVector3(originVelocity),
      baselineYaw: baseline.yaw,
      baselinePitch: baseline.pitch,
      providedYawPitchSeed,
      providedYaw: providedYawCheck.solution?.yawRadians,
      providedPitch: providedYawCheck.solution?.pitchRadians,
      heuristicIterations: getHeuristicIterations(providedYawCheck.solution),
      collisionFallbackIterations: getCollisionFallbackIterations(providedYawCheck.solution),
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(providedYawCheck.solution, "collision fallback replay should still find a hit");
    assert.equal(providedYawCheck.solution?.hit, true);
    assert.ok(
      Math.abs((providedYawCheck.solution?.yawRadians ?? 0) - baseline.yaw) <= 1e-9,
      "collision fallback replay should preserve the baseline yaw exactly",
    );
    assert.ok(
      Math.abs((providedYawCheck.solution?.pitchRadians ?? 0) - baseline.pitch) <= 0.2,
      "collision fallback pitch should stay reasonably close to mineflayer-ender output",
    );
    assert.ok(
      getClosestPointDistance(
        providedYawCheck.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(baselineDiagnostics.result.closestPoint),
      ) <= 0.5,
      "collision fallback intercept point should stay close to mineflayer-ender output",
    );
  });
});

test("new solver: replay alt-seed case successfully finds high arc angle", () => {
  const fixture = createReplayFixture(new Vec3(-79.0, 4.0, -39.5));
  const baselineStartedAt = performance.now();
  const baseline = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
  )!;
  const openArc = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
    undefined,
    20,
  )!;
  const baselineElapsedMs = performance.now() - baselineStartedAt;
  const originVelocity = getOriginVelocity(fixture.bot);
  const providedYawPitchSeed = Math.PI / 2 - 0.1;
  const providedYawCheck = solveReplayAimWithCollisionFallbackPitchHeuristic(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
      createCandidateGenerator(
      Math.PI / 2,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
    undefined,
    fixture.enderman.maxTicks,
  );

  // console.log('bot position', toVector3(fixture.bot.entity.position));
  // console.log('target position', toVector3(fixture.landingPos));
  // console.log('baseline pitch', baseline.pitch, "radians", baseline.pitch * (180 / Math.PI), "degrees");
  // console.log('open arc pitch', openArc.pitch, "radians", openArc.pitch * (180 / Math.PI), "degrees");
  // console.log(providedYawCheck.solution?.pitchRadians ? providedYawCheck.solution?.pitchRadians * (180 / Math.PI) : 0, "degrees");
  // console.log(providedYawCheck.solution)

  const baselineDiagnostics = createShotDiagnostics(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    baseline.yaw,
    baseline.pitch,
    originVelocity,
  );

  printReplayMetrics({
    label: "alt-arc-seed",
    baselineElapsedMs,
    solveElapsedMs: providedYawCheck.elapsedMs,
    iterations: getHeuristicIterations(providedYawCheck.solution),
    collisionFreeIterations: getCollisionFallbackIterations(providedYawCheck.solution).collisionFreeIterations,
    collisionAwareEvaluations: getCollisionFallbackIterations(providedYawCheck.solution).collisionAwareEvaluations,
  });

  const printDetails = logReplayValues({
    enabled: false,
    onlyOnFailure: SHOW_REPLAY_VALUES_ON_FAILURE,
    baselineElapsedMs,
    providedYawElapsedMs: providedYawCheck.elapsedMs,
    summary: {
      originVelocity: toVector3(originVelocity),
      baselineYaw: baseline.yaw,
      baselinePitch: baseline.pitch,
      providedYawPitchSeed,
      providedYaw: providedYawCheck.solution?.yawRadians,
      providedPitch: providedYawCheck.solution?.pitchRadians,
      heuristicIterations: getHeuristicIterations(providedYawCheck.solution),
      collisionFallbackIterations: getCollisionFallbackIterations(providedYawCheck.solution),
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      openArcPitch: openArc.pitch,
      openArcClosestPosition: toVector3(openArc.shotInfo.closestPoint),
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });


  runWithFailureLogging(printDetails, () => {
    assert.ok(openArc?.hit, "open-world delayed replay should produce a high-arc hit");
    assert.ok(providedYawCheck.solution, "collision fallback alt replay should still find a hit");
    assert.equal(providedYawCheck.solution?.hit, true);
    assert.ok(
      Math.abs((providedYawCheck.solution?.yawRadians ?? 0) - openArc.yaw) <= 1e-9,
      "collision fallback alt replay should preserve the arc yaw",
    );
    assert.ok(
      Math.abs((providedYawCheck.solution?.pitchRadians ?? 0) - openArc.pitch) <= 0.1,
      "collision fallback alt replay should converge to the high-arc pitch",
    );
    assert.ok(
      getClosestPointDistance(
        providedYawCheck.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(openArc.shotInfo.closestPoint),
      ) <= 0.25,
      "collision fallback alt replay should stay close to the delayed high-arc intercept",
    );
  });
});

test("new solver: blocked straight-shot replay recovers an alternate arc", () => {
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

  assert.ok(openBaseline?.hit, "open-world baseline should produce a hit");
  assert.ok(openArc?.hit, "open-world delayed replay should still produce a hit");

  const blockedFixture = createReplayFixture(landingPos, {
    bot: createReplayBot({ solidBlocks: blockingBlocks }),
  });
  const blockedOriginVelocity = getOriginVelocity(blockedFixture.bot);
  const blockedStraightDiagnostics = createShotDiagnostics(
    blockedFixture.bot,
    blockedFixture.landingAABB,
    blockedFixture.landingPos,
    openBaseline.yaw,
    openBaseline.pitch,
    blockedOriginVelocity,
  );
  const recoveredShot = solveReplayAimWithCollisionFallbackPitchHeuristic(
    blockedFixture.bot,
    blockedFixture.landingAABB,
    blockedFixture.landingPos,
    blockedOriginVelocity,
    createCandidateGenerator(
      openBaseline.pitch,
      { provideYaw: () => openBaseline.yaw },
    ).getCandidates(blockedFixture.replayContext),
    createCandidateGenerator(
      openArc.pitch,
      { provideYaw: () => openArc.yaw },
    ).getCandidates(blockedFixture.replayContext),
    undefined,
    blockedFixture.enderman.maxTicks,
  );

  printReplayMetrics({
    label: "collision-fallback",
    baselineElapsedMs: 0,
    solveElapsedMs: recoveredShot.elapsedMs,
    iterations: getHeuristicIterations(recoveredShot.solution),
    collisionFreeIterations: getCollisionFallbackIterations(recoveredShot.solution).collisionFreeIterations,
    collisionAwareEvaluations: getCollisionFallbackIterations(recoveredShot.solution).collisionAwareEvaluations,
  });

  const printDetails = logReplayValues({
    enabled: false,
    onlyOnFailure: SHOW_REPLAY_VALUES_ON_FAILURE,
    baselineElapsedMs: 0,
    providedYawElapsedMs: recoveredShot.elapsedMs,
    summary: {
      blockingBlocks: blockingBlocks.map((block) => toVector3(block)),
      openBaselineYaw: openBaseline.yaw,
      openBaselinePitch: openBaseline.pitch,
      openArcYaw: openArc.yaw,
      openArcPitch: openArc.pitch,
      blockedStraight: {
        hit: !!blockedStraightDiagnostics.result.hit,
        pitch: openBaseline.pitch,
        closestPosition: toVector3(blockedStraightDiagnostics.result.closestPoint),
      },
      recoveredShot: recoveredShot.solution && {
        hit: recoveredShot.solution.hit,
        pitch: recoveredShot.solution.pitchRadians,
        closestPosition: recoveredShot.solution.closestPosition,
        heuristicIterations: getHeuristicIterations(recoveredShot.solution),
        collisionFallbackIterations: getCollisionFallbackIterations(recoveredShot.solution),
      },
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(
      !blockedStraightDiagnostics.result.hit,
      "the original straight shot should no longer hit once the block is added",
    );
    assert.ok(recoveredShot.solution, "collision fallback replay should still find a solution");
    assert.equal(recoveredShot.solution?.hit, true);
    assert.ok(
      Math.abs((recoveredShot.solution?.yawRadians ?? 0) - openBaseline.yaw) <= 1e-9,
      "collision fallback replay should preserve the original yaw",
    );
    assert.ok(
      Math.abs((recoveredShot.solution?.pitchRadians ?? 0) - openBaseline.pitch) >= 0.1,
      "collision fallback replay should recover a meaningfully different pitch from the blocked straight shot",
    );
    assert.ok(
      getClosestPointDistance(
        recoveredShot.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(openArc.shotInfo.closestPoint),
      ) <= 0.25,
      "collision fallback replay should converge to the user-supplied alternate arc shot",
    );
  });
});

test("new solver: blocked straight-shot replay identifies the alternate angle candidate", () => {
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

  assert.ok(openBaseline?.hit, "open-world baseline should produce a hit");
  assert.ok(openArc?.hit, "open-world delayed replay should still produce a hit");

  const blockedFixture = createReplayFixture(landingPos, {
    bot: createReplayBot({ solidBlocks: blockingBlocks }),
  });
  const blockedOriginVelocity = getOriginVelocity(blockedFixture.bot);
  const blockedStraightDiagnostics = createShotDiagnostics(
    blockedFixture.bot,
    blockedFixture.landingAABB,
    blockedFixture.landingPos,
    openBaseline.yaw,
    openBaseline.pitch,
    blockedOriginVelocity,
  );
  const recoveredShot = solveReplayAimWithCollisionFallbackPitchHeuristic(
    blockedFixture.bot,
    blockedFixture.landingAABB,
    blockedFixture.landingPos,
    blockedOriginVelocity,
    createCandidateGenerator(
      openBaseline.pitch,
      { provideYaw: () => openBaseline.yaw },
    ).getCandidates(blockedFixture.replayContext),
    createCandidateGenerator(
      openArc.pitch,
      { provideYaw: () => openArc.yaw },
    ).getCandidates(blockedFixture.replayContext),
    undefined,
    blockedFixture.enderman.maxTicks,
  );
  const fallbackMetrics = getCollisionFallbackIterations(recoveredShot.solution);
  const [initialStage, alternateStage] = fallbackMetrics.stages ?? [];

  const printDetails = logReplayValues({
    enabled: false,
    onlyOnFailure: SHOW_REPLAY_VALUES_ON_FAILURE,
    baselineElapsedMs: 0,
    providedYawElapsedMs: recoveredShot.elapsedMs,
    summary: {
      blockingBlocks: blockingBlocks.map((block) => toVector3(block)),
      openBaselineYaw: openBaseline.yaw,
      openBaselinePitch: openBaseline.pitch,
      openArcYaw: openArc.yaw,
      openArcPitch: openArc.pitch,
      blockedStraight: {
        hit: !!blockedStraightDiagnostics.result.hit,
        pitch: openBaseline.pitch,
        closestPosition: toVector3(blockedStraightDiagnostics.result.closestPoint),
      },
      collisionFallbackMetrics: fallbackMetrics,
      recoveredShot: recoveredShot.solution && {
        hit: recoveredShot.solution.hit,
        yaw: recoveredShot.solution.yawRadians,
        pitch: recoveredShot.solution.pitchRadians,
        closestPosition: recoveredShot.solution.closestPosition,
      },
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(
      !blockedStraightDiagnostics.result.hit,
      "the original straight shot should be blocked before fallback runs",
    );
    assert.ok(recoveredShot.solution, "collision fallback replay should still find a solution");
    assert.equal(recoveredShot.solution?.hit, true);
    assert.equal(
      fallbackMetrics.attemptedCandidates,
      2,
      "collision fallback should evaluate the blocked straight-shot seed and the alternate-angle seed",
    );
    assert.equal(
      fallbackMetrics.collisionAwareEvaluations,
      2,
      "collision fallback should validate both the blocked straight-shot and alternate-angle candidates",
    );
    assert.equal(initialStage?.kind, "initial");
    assert.equal(
      initialStage?.collisionAwareHit,
      false,
      "the initial straight-shot candidate should fail once collisions are considered",
    );
    assert.equal(alternateStage?.kind, "alternate");
    assert.equal(
      alternateStage?.collisionAwareHit,
      true,
      "the alternate-angle candidate should be the collision-aware hit that gets selected",
    );
    assert.ok(
      Math.abs((alternateStage?.seedPitchRadians ?? 0) - openArc.pitch) <= 1e-9,
      "the successful alternate stage should be seeded from the known alternate arc angle",
    );
    assert.ok(
      Math.abs((recoveredShot.solution?.pitchRadians ?? 0) - openArc.pitch) <= 0.1,
      "the recovered solution should converge to the alternate arc angle",
    );
  });
});

test("new solver: high-arc replay case with minFlightTicks can be evaluated through projectile-aim", () => {
  const fixture = createReplayFixture(new Vec3(-121.0, 4.0, -29.5), {
    maxTicks: REPLAY_TEST_MAX_TICKS,
  });
  const minFlightTicks = 30;
  const baselineStartedAt = performance.now();
  const baseline = fixture.enderman.shotToAABB(
    fixture.landingAABB,
    fixture.landingPos,
    undefined,
    minFlightTicks,
  )!;
  const baselineElapsedMs = performance.now() - baselineStartedAt;
  const originVelocity = getOriginVelocity(fixture.bot);

  assert.ok(baseline?.hit, "mineflayer-ender high-arc baseline replay should produce a hit");
  assert.ok(
    baseline.ticks >= minFlightTicks,
    "mineflayer-ender high-arc baseline replay should respect minFlightTicks",
  );

  const providedYawPitchSeed = estimatePitchWithLinearDrag(
    fixture.bot,
    baseline.yaw,
    toVector3(fixture.landingPos),
    originVelocity,
  );
  const providedYawCheck = solveReplayAimWithCollisionFallbackPitchHeuristic(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
    undefined,
    undefined,
    fixture.enderman.maxTicks,
  );
  const baselineDiagnostics = createShotDiagnostics(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    baseline.yaw,
    baseline.pitch,
    originVelocity,
  );

  printReplayMetrics({
    label: "high-arc",
    baselineElapsedMs,
    solveElapsedMs: providedYawCheck.elapsedMs,
    iterations: getHeuristicIterations(providedYawCheck.solution),
    collisionFreeIterations: getCollisionFallbackIterations(providedYawCheck.solution).collisionFreeIterations,
    collisionAwareEvaluations: getCollisionFallbackIterations(providedYawCheck.solution).collisionAwareEvaluations,
  });

  const printDetails = logReplayValues({
    enabled: false,
    onlyOnFailure: SHOW_REPLAY_VALUES_ON_FAILURE,
    baselineElapsedMs,
    providedYawElapsedMs: providedYawCheck.elapsedMs,
    summary: {
      minFlightTicks,
      originVelocity: toVector3(originVelocity),
      baselineYaw: baseline.yaw,
      baselinePitch: baseline.pitch,
      baselineTicks: baseline.ticks,
      providedYawPitchSeed,
      providedYaw: providedYawCheck.solution?.yawRadians,
      providedPitch: providedYawCheck.solution?.pitchRadians,
      heuristicIterations: getHeuristicIterations(providedYawCheck.solution),
      collisionFallbackIterations: getCollisionFallbackIterations(providedYawCheck.solution),
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(
      providedYawCheck.solution,
      "collision fallback high-arc replay should still find a solution",
    );
    assert.equal(providedYawCheck.solution?.hit, true);
    assert.ok(
      Math.abs((providedYawCheck.solution?.yawRadians ?? 0) - baseline.yaw) <= 1e-9,
      "collision fallback high-arc replay should preserve mineflayer-ender yaw exactly",
    );
    assert.ok(
      Math.abs((providedYawCheck.solution?.pitchRadians ?? 0) - baseline.pitch) <= 0.2,
      "collision fallback high-arc pitch should stay reasonably close to mineflayer-ender output",
    );
    assert.ok(
      getClosestPointDistance(
        providedYawCheck.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(baselineDiagnostics.result.closestPoint),
      ) <= 0.75,
      "collision fallback intercept point should stay close to mineflayer-ender output",
    );
  });
});
