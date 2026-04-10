const setup = require("./test/mineflayer/setup.ts");
const utils = require("./test/mineflayer/enderTestUtils.ts");
const index = require("./src/index.ts");

function getHeuristicHistory(solution) {
  return solution?.candidate?.metadata?.pitchHeuristic?.history ?? [];
}

function getClosestSampleDirection(samples, closestSample) {
  const closestIndex = samples.findIndex((sample) => sample === closestSample);
  const currentSample = closestIndex >= 0 ? samples[closestIndex] : closestSample;

  if (currentSample?.velocity) {
    return currentSample.velocity;
  }

  const previousSample = closestIndex > 0 ? samples[closestIndex - 1] : null;
  const nextSample =
    closestIndex >= 0 && closestIndex < samples.length - 1
      ? samples[closestIndex + 1]
      : null;

  if (previousSample && nextSample) {
    return index.subtractVectors(nextSample.position, previousSample.position);
  }

  if (nextSample) {
    return index.subtractVectors(nextSample.position, currentSample.position);
  }

  if (previousSample) {
    return index.subtractVectors(currentSample.position, previousSample.position);
  }

  return { x: 0, y: 0, z: 0 };
}

function getTravelState(context, evaluation, samples) {
  const targetPoint =
    context.target.idealPoint ??
    context.target.geometry.idealPoint ??
    context.target.referencePoint ??
    context.target.geometry.referencePoint ??
    context.origin;
  const closestPoint = evaluation.hitPoint ?? evaluation.closestSample.position;
  const velocity = getClosestSampleDirection(samples, evaluation.closestSample);
  const toTarget = index.subtractVectors(targetPoint, closestPoint);
  const direction =
    velocity.x === 0 && velocity.y === 0 && velocity.z === 0
      ? index.subtractVectors(closestPoint, context.origin)
      : velocity;

  return index.dotVectors(direction, toTarget) >= 0 ? "undershoot" : "overshoot";
}

function getOriginVelocity(bot) {
  return bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
}

function formatPoint(point) {
  if (!point) {
    return null;
  }

  return {
    x: Number(point.x.toFixed(6)),
    y: Number(point.y.toFixed(6)),
    z: Number(point.z.toFixed(6)),
  };
}

function summarizeHistory(history) {
  return history.map((entry) => ({
    iteration: entry.iteration,
    pitch: Number(entry.pitchRadians.toFixed(6)),
    delta: Number(entry.deltaRadians.toFixed(6)),
    distance: Number(entry.distance.toFixed(6)),
    hit: entry.hit,
  }));
}

function evaluatePitch(context, provider, pitchRadians, yawRadians) {
  const trajectory = index.normalizeTrajectorySource(
    provider.getTrajectory(context, {
      yawRadians,
      pitchRadians,
    }),
  );
  const samples = Array.isArray(trajectory.samples)
    ? trajectory.samples
    : Array.from(trajectory.samples);
  const evaluation = index.evaluateTrajectory(context, samples);

  if (!evaluation) {
    return null;
  }

  const closestPoint = evaluation.hitPoint ?? evaluation.closestSample.position;
  const targetPoint =
    context.target.idealPoint ??
    context.target.geometry.idealPoint ??
    context.target.referencePoint ??
    context.target.geometry.referencePoint ??
    context.origin;
  const qualityDistance = evaluation.hit
    ? index.getDistance(closestPoint, targetPoint)
    : evaluation.distance;

  return {
    hit: evaluation.hit,
    qualityDistance: Number(qualityDistance.toFixed(6)),
    closestPosition: formatPoint(closestPoint),
    closestTick: evaluation.closestSample.tick,
    travelState: getTravelState(context, evaluation, samples),
    sampleCount: samples.length,
  };
}

function compareHistories(withCollisions, withoutCollisions, context, providerFactory, yawRadians) {
  const maxLength = Math.max(withCollisions.length, withoutCollisions.length);
  const rows = [];

  for (let index = 0; index < maxLength; index += 1) {
    const a = withCollisions[index] ?? null;
    const b = withoutCollisions[index] ?? null;
    const withProvider = providerFactory(true);
    const withoutProvider = providerFactory(false);

    rows.push({
      iteration: index,
      withCollisions: a
        ? {
            ...a,
            analysis: evaluatePitch(
              context,
              withProvider,
              a.pitch,
              yawRadians,
            ),
          }
        : null,
      withoutCollisions: b
        ? {
            ...b,
            analysis: evaluatePitch(
              context,
              withoutProvider,
              b.pitch,
              yawRadians,
            ),
          }
        : null,
    });
  }

  return rows;
}

function runScenario(input) {
  const bot = setup.createReplayBot(input.botOptions);
  const enderman = new setup.Enderman(bot);
  const landingAABB = setup.AABBUtils.getEntityAABBRaw({
    position: input.landingPos,
    height: 1.8,
    width: 0.6,
  });
  const replayContext = utils.createReplayContext(
    setup.toVector3(bot.entity.position),
    landingAABB,
    setup.toVector3(input.landingPos),
  );
  const originVelocity = getOriginVelocity(bot);

  enderman.maxTicks = input.maxTicks ?? 100;
  enderman.dvStep = input.dvStep ?? 360;

  const baseline = enderman.shotToAABB(
    landingAABB,
    input.landingPos,
    undefined,
    input.minFlightTicks,
  );
  const provider = utils.createTrajectoryProvider(
    bot,
    landingAABB,
    input.landingPos,
    originVelocity,
    input.blockChecking,
  );
  const solution = index.solveAimWithPitchHeuristic({
    context: replayContext,
    provider,
    candidates: utils.createCandidateGenerator(input.pitchSeed, {
      provideYaw: () => input.yawRadians,
    }).getCandidates(replayContext),
    initialDeltaRadians: input.initialDeltaRadians ?? 0.04,
    maxIterations: input.maxIterations ?? 30,
  });

  const shotDiagnostics = solution
    ? (() => {
        const shot = setup.EnderShotFactory.fromPlayer(
          {
            position: bot.entity.position,
            yaw: solution.yawRadians,
            pitch: solution.pitchRadians,
            velocity: originVelocity,
          },
          bot,
        );
        const result = shot.calcToAABB(
          landingAABB,
          input.landingPos,
          input.blockChecking,
        );

        return {
          shot,
          result,
        };
      })()
    : null;

  return {
    replayContext,
    providerFactory: (blockChecking) =>
      utils.createTrajectoryProvider(
        bot,
        landingAABB,
        input.landingPos,
        originVelocity,
        blockChecking,
      ),
    blockChecking: input.blockChecking,
    baseline: baseline
      ? {
          hit: baseline.hit,
          yaw: Number(baseline.yaw.toFixed(6)),
          pitch: Number(baseline.pitch.toFixed(6)),
          ticks: baseline.ticks,
          closestPoint: formatPoint(
            baseline.shotInfo?.closestPoint
              ? setup.toVector3(baseline.shotInfo.closestPoint)
              : null,
          ),
        }
      : null,
    solution: solution
      ? {
          hit: solution.hit,
          yawRadians: Number(solution.yawRadians.toFixed(6)),
          pitchRadians: Number(solution.pitchRadians.toFixed(6)),
          closestPosition: formatPoint(solution.closestPosition),
          distance: Number(solution.distance.toFixed(6)),
          tick: solution.tick,
        }
      : null,
    diagnostic: shotDiagnostics
      ? {
          sampledPoints: shotDiagnostics.shot.points.length,
          closestPoint: formatPoint(
            shotDiagnostics.result.closestPoint
              ? setup.toVector3(shotDiagnostics.result.closestPoint)
              : null,
          ),
          hit: shotDiagnostics.result.hit,
        }
      : null,
    history: summarizeHistory(getHeuristicHistory(solution)),
  };
}

const scenarios = [
  {
    name: "baseline-replay-seed",
    landingPos: new setup.Vec3(-79.5, 4.0, -29.5),
    pitchSeed: -0.28372138980855915,
    yawRadians: 2.120331882934733,
  },
//   {
//     name: "blocked-straight-seed",
//     landingPos: new setup.Vec3(-79.0, 4.0, -39.5),
//     botOptions: {
//       solidBlocks: [new setup.Vec3(-77, 4, -36), new setup.Vec3(-77, 5, -36)],
//     },
//     pitchSeed: -0.12217304763960825,
//     yawRadians: 0.6168100829247134,
//   },
];

for (const scenario of scenarios) {
  const withCollisions = runScenario({
    ...scenario,
    blockChecking: true,
  });
  const withoutCollisions = runScenario({
    ...scenario,
    blockChecking: false,
  });

  console.log(`=== ${scenario.name} ===`);
  console.log(
    JSON.stringify(
      {
        final: {
          withCollisions: {
            solution: withCollisions.solution,
            diagnostic: withCollisions.diagnostic,
          },
          withoutCollisions: {
            solution: withoutCollisions.solution,
            diagnostic: withoutCollisions.diagnostic,
          },
        },
        deltaWalk: compareHistories(
          withCollisions.history,
          withoutCollisions.history,
          withCollisions.replayContext,
          withCollisions.providerFactory,
          scenario.yawRadians,
        ),
      },
      null,
      2,
    ),
  );
  console.log("");
}
