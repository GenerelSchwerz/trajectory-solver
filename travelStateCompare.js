const setup = require("./test/mineflayer/setup.ts");
const utils = require("./test/mineflayer/enderTestUtils.ts");
const index = require("./src/index.ts");

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

function getReferencePoint(context) {
  return (
    context.target.referencePoint ??
    context.target.geometry.referencePoint ??
    context.target.idealPoint ??
    context.target.geometry.idealPoint ??
    context.origin
  );
}

function getTravelStateDetails(context, samples, evaluation) {
  const closestPoint = evaluation.hitPoint ?? evaluation.closestSample.position;
  const targetPoint = getReferencePoint(context);
  const velocity = getClosestSampleDirection(samples, evaluation.closestSample);
  const toTarget = index.subtractVectors(targetPoint, closestPoint);
  const direction =
    velocity.x === 0 && velocity.y === 0 && velocity.z === 0
      ? index.subtractVectors(closestPoint, context.origin)
      : velocity;
  const dot = index.dotVectors(direction, toTarget);

  return {
    state: dot >= 0 ? "undershoot" : "overshoot",
    dot: Number(dot.toFixed(6)),
    closestPoint: formatPoint(closestPoint),
    closestTick: evaluation.closestSample.tick,
    direction: formatPoint(direction),
    toTarget: formatPoint(toTarget),
    sampleCount: samples.length,
    hit: evaluation.hit,
  };
}

function evaluatePitchSet(input) {
  const bot = setup.createReplayBot(input.botOptions);
  const landingAABB = setup.AABBUtils.getEntityAABBRaw({
    position: input.landingPos,
    height: 1.8,
    width: 0.6,
  });
  const context = utils.createReplayContext(
    setup.toVector3(bot.entity.position),
    landingAABB,
    setup.toVector3(input.landingPos),
  );
  const originVelocity = getOriginVelocity(bot);
  const provider = utils.createTrajectoryProvider(
    bot,
    landingAABB,
    input.landingPos,
    originVelocity,
    input.blockChecking,
  );

  return input.pitches.map((pitchRadians) => {
    const trajectory = index.normalizeTrajectorySource(
      provider.getTrajectory(context, {
        yawRadians: input.yawRadians,
        pitchRadians,
      }),
    );
    const samples = Array.isArray(trajectory.samples)
      ? trajectory.samples
      : Array.from(trajectory.samples);
    const evaluation = index.evaluateTrajectory(context, samples);

    return {
      pitchRadians: Number(pitchRadians.toFixed(6)),
      details: evaluation
        ? getTravelStateDetails(context, samples, evaluation)
        : null,
    };
  });
}

const scenario = {
  name: "baseline-replay-seed",
  landingPos: new setup.Vec3(-79.5, 4.0, -29.5),
  yawRadians: 2.120331882934733,
  pitches: [
    -0.28372138980855915,
    -0.24372138980855915,
    -0.20372138980855915,
    -0.16372138980855915,
    -0.12372138980855915,
    -0.08372138980855915,
  ],
};

console.log(
  JSON.stringify(
    {
      scenario: scenario.name,
      withCollisions: evaluatePitchSet({
        ...scenario,
        blockChecking: true,
      }),
      withoutCollisions: evaluatePitchSet({
        ...scenario,
        blockChecking: false,
      }),
    },
    null,
    2,
  ),
);
