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
  solveReplayAim,
  solveReplayAimWithPitchHeuristic,
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

function getHeuristicIterations(solution: { candidate?: { metadata?: Record<string, unknown> } } | null | undefined) {
  return (
    (
      solution?.candidate?.metadata?.pitchHeuristic as
        | { history?: unknown[] }
        | undefined
    )?.history?.length ?? 0
  );
}

function printReplayMetrics(input: {
  label: string;
  baselineElapsedMs: number;
  solveElapsedMs: number;
  iterations?: number;
}) {
  const parts = [
    `[old:${input.label}] baseline=${input.baselineElapsedMs.toFixed(3)}ms`,
    `solve=${input.solveElapsedMs.toFixed(3)}ms`,
  ];

  if (input.iterations !== undefined) {
    parts.push(`iterations=${input.iterations}`);
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

test("old solver: replay case can be evaluated through projectile-aim", () => {
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
  const providedYawCheck = solveReplayAimWithPitchHeuristic(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
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
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(providedYawCheck.solution, "provided mineflayer-ender yaw check should still find a hit");
    assert.equal(providedYawCheck.solution?.hit, true);
    assert.ok(
      Math.abs((providedYawCheck.solution?.yawRadians ?? 0) - baseline.yaw) <= 1e-9,
      "provided yaw check should preserve mineflayer-ender yaw exactly",
    );
    assert.ok(
      Math.abs((providedYawCheck.solution?.pitchRadians ?? 0) - baseline.pitch) <= 0.2,
      "provided yaw pitch should stay reasonably close to mineflayer-ender output",
    );
    assert.ok(
      getClosestPointDistance(
        providedYawCheck.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(baselineDiagnostics.result.closestPoint),
      ) <= 0.5,
      "provided yaw intercept point should stay close to mineflayer-ender output",
    );
    assert.ok(
      providedYawCheck.elapsedMs <= baselineElapsedMs * 2,
      "heuristic solve should stay in the same rough performance range as the baseline replay solve",
    );
  });
});


test("old solver: high-arc replay case with minFlightTicks can be evaluated through projectile-aim", () => {
  const fixture = createReplayFixture(new Vec3(-121.0, 4.0, -29.5), {
    maxTicks: 140,
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
  const providedYawCheck = solveReplayAimWithPitchHeuristic(
    fixture.bot,
    fixture.landingAABB,
    fixture.landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(fixture.replayContext),
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
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

  runWithFailureLogging(printDetails, () => {
    assert.ok(
      providedYawCheck.solution,
      "provided mineflayer-ender high-arc yaw check should still find a solution",
    );
    assert.equal(providedYawCheck.solution?.hit, true);
    assert.ok(
      Math.abs((providedYawCheck.solution?.yawRadians ?? 0) - baseline.yaw) <= 1e-9,
      "provided high-arc yaw check should preserve mineflayer-ender yaw exactly",
    );
    assert.ok(
      Math.abs((providedYawCheck.solution?.pitchRadians ?? 0) - baseline.pitch) <= 0.2,
      "provided high-arc yaw pitch should stay reasonably close to mineflayer-ender output",
    );
    assert.ok(
      getClosestPointDistance(
        providedYawCheck.solution?.closestPosition ?? { x: 0, y: 0, z: 0 },
        toVector3(baselineDiagnostics.result.closestPoint),
      ) <= 0.75,
      "provided high-arc yaw intercept point should stay close to mineflayer-ender output",
    );
  });
});
