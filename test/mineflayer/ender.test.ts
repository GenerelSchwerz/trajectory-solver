import assert from "node:assert/strict";
import { test } from "node:test";
import { performance } from "node:perf_hooks";

import {
  AABBUtils,
  createReplayBot,
  Enderman,
  toVector3,
  Vec3,
} from "./setup";
import {
  createCandidateGenerator,
  createReplayContext,
  createShotDiagnostics,
  estimatePitchWithLinearDrag,
  getClosestPointDistance,
  logReplayValues,
  solveReplayAimWithPitchHeuristic,
} from "./enderTestUtils";

const SHOW_REPLAY_VALUES = process.env.SHOW_REPLAY_VALUES === "true";

test("mineflayer ender replay case can be evaluated through projectile-aim", () => {
  const bot = createReplayBot();
  const enderman = new Enderman(bot);
  const landingPos = new Vec3(-79.5, 4.0, -29.5);
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

  enderman.maxTicks = 100;
  enderman.dvStep = 360;

  const baselineStartedAt = performance.now();
  const baseline = enderman.shotToAABB(landingAABB, landingPos)!;
  const baselineElapsedMs = performance.now() - baselineStartedAt;
  assert.ok(baseline?.hit, "mineflayer-ender baseline replay should produce a hit");

  const originVelocity = bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
    
  const providedYawPitchSeed = estimatePitchWithLinearDrag(
    bot,
    baseline.yaw,
    toVector3(landingPos),
    originVelocity,
  );

  const providedYawCheck = solveReplayAimWithPitchHeuristic(
    bot,
    landingAABB,
    landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(replayContext),
  );

  const baselineDiagnostics = createShotDiagnostics(
    bot,
    landingAABB,
    landingPos,
    baseline.yaw,
    baseline.pitch,
    originVelocity,
  );
  logReplayValues({
    enabled: SHOW_REPLAY_VALUES,
    baselineElapsedMs,
    providedYawElapsedMs: providedYawCheck.elapsedMs,
    summary: {
      originVelocity: toVector3(originVelocity),
      baselineYaw: baseline.yaw,
      baselinePitch: baseline.pitch,
      providedYawPitchSeed,
      providedYaw: providedYawCheck.solution?.yawRadians,
      providedPitch: providedYawCheck.solution?.pitchRadians,
      heuristicIterations:
        (
          providedYawCheck.solution?.candidate.metadata?.pitchHeuristic as
            | { history?: unknown[] }
            | undefined
        )?.history?.length ?? 0,
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

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

test("mineflayer ender high-arc replay case with minFlightTicks can be evaluated through projectile-aim", () => {
  const bot = createReplayBot();
  const enderman = new Enderman(bot);
  const landingPos = new Vec3(-121.0, 4.0, -29.5);
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
  const minFlightTicks = 30;

  enderman.maxTicks = 140;
  enderman.dvStep = 360;

  const baselineStartedAt = performance.now();
  const baseline = enderman.shotToAABB(
    landingAABB,
    landingPos,
    undefined,
    minFlightTicks,
  )!;
  const baselineElapsedMs = performance.now() - baselineStartedAt;
  assert.ok(baseline?.hit, "mineflayer-ender high-arc baseline replay should produce a hit");
  assert.ok(
    baseline.ticks >= minFlightTicks,
    "mineflayer-ender high-arc baseline replay should respect minFlightTicks",
  );

  const originVelocity = bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);

  const providedYawPitchSeed = estimatePitchWithLinearDrag(
    bot,
    baseline.yaw,
    toVector3(landingPos),
    originVelocity,
  );

  const providedYawCheck = solveReplayAimWithPitchHeuristic(
    bot,
    landingAABB,
    landingPos,
    originVelocity,
    createCandidateGenerator(
      providedYawPitchSeed,
      { provideYaw: () => baseline.yaw },
    ).getCandidates(replayContext),
  );

  const baselineDiagnostics = createShotDiagnostics(
    bot,
    landingAABB,
    landingPos,
    baseline.yaw,
    baseline.pitch,
    originVelocity,
  );
  logReplayValues({
    enabled: SHOW_REPLAY_VALUES,
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
      heuristicIterations:
        (
          providedYawCheck.solution?.candidate.metadata?.pitchHeuristic as
            | { history?: unknown[] }
            | undefined
        )?.history?.length ?? 0,
      providedYawClosestPosition: providedYawCheck.solution?.closestPosition,
      baselineClosestPosition: toVector3(baselineDiagnostics.result.closestPoint),
    },
  });

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
