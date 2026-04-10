const setup = require("./test/mineflayer/setup.ts");
const utils = require("./test/mineflayer/enderTestUtils.ts");
const index = require("./src/index.ts");

function getOriginVelocity(bot) {
  return bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
}

function formatNumber(value) {
  return Number(value.toFixed(6));
}

function formatPoint(point) {
  if (!point) {
    return null;
  }

  return {
    x: formatNumber(point.x),
    y: formatNumber(point.y),
    z: formatNumber(point.z),
  };
}

function roundPitchValue(value) {
  return Number(value.toFixed(12));
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

function getIdealPoint(context) {
  return context.target.idealPoint ?? context.target.geometry.idealPoint ?? null;
}

function getAimQualityDistance(context, evaluation) {
  const idealPoint = getIdealPoint(context);

  if (!idealPoint || !evaluation.hit) {
    return evaluation.distance;
  }

  return index.getDistance(
    evaluation.hitPoint ?? evaluation.closestSample.position,
    idealPoint,
  );
}

function getHorizontalProgress(origin, targetPoint, point) {
  const targetVector = {
    x: targetPoint.x - origin.x,
    z: targetPoint.z - origin.z,
  };
  const targetAlong = Math.hypot(targetVector.x, targetVector.z);

  if (targetAlong <= 1e-6) {
    return null;
  }

  const unit = {
    x: targetVector.x / targetAlong,
    z: targetVector.z / targetAlong,
  };
  const pointVector = {
    x: point.x - origin.x,
    z: point.z - origin.z,
  };

  return {
    along: pointVector.x * unit.x + pointVector.z * unit.z,
    targetAlong,
  };
}

function getPitchTravelState(context, samples) {
  const targetPoint = getIdealPoint(context) ?? getReferencePoint(context);
  const progresses = samples
    .map((sample) => ({
      sample,
      progress: getHorizontalProgress(context.origin, targetPoint, sample.position),
    }))
    .filter((entry) => entry.progress !== null);

  if (progresses.length === 0) {
    return "undershoot";
  }

  const targetAlong = progresses[0].progress.targetAlong;

  for (let i = 1; i < progresses.length; i += 1) {
    const previous = progresses[i - 1];
    const current = progresses[i];

    if (
      previous.progress.along > targetAlong + 1e-6 ||
      current.progress.along < targetAlong - 1e-6
    ) {
      continue;
    }

    const alongDelta = current.progress.along - previous.progress.along;
    const interpolation =
      Math.abs(alongDelta) <= 1e-6
        ? 1
        : (targetAlong - previous.progress.along) / alongDelta;
    const alignedY =
      previous.sample.position.y +
      (current.sample.position.y - previous.sample.position.y) * interpolation;

    return alignedY > targetPoint.y + 1e-6 ? "overshoot" : "undershoot";
  }

  const nearest = progresses.reduce((best, current) =>
    Math.abs(current.progress.along - targetAlong) <
    Math.abs(best.progress.along - targetAlong)
      ? current
      : best,
  );

  return nearest.sample.position.y > targetPoint.y + 1e-6
    ? "overshoot"
    : "undershoot";
}

function evaluateWithTerminalPoint(context, trajectory) {
  const samples = Array.from(trajectory.samples);
  const evaluation = index.evaluateTrajectory(context, samples);

  if (evaluation?.hit) {
    return {
      evaluation,
      samples,
      closestPosition: evaluation.hitPoint ?? evaluation.closestSample.position,
    };
  }

  const terminalPoint = trajectory.terminalPoint ?? null;

  if (!terminalPoint) {
    return {
      evaluation,
      samples,
      closestPosition: evaluation
        ? evaluation.closestSample.position
        : null,
    };
  }

  const terminalDistance =
    context.target.geometry.distanceTo?.(terminalPoint.position) ??
    index.getDistance(
      terminalPoint.position,
      getIdealPoint(context) ?? getReferencePoint(context),
    );
  const currentDistance = evaluation?.distance ?? Number.POSITIVE_INFINITY;

  if (terminalDistance < currentDistance) {
    return {
      evaluation: {
        hit: false,
        distance: terminalDistance,
        closestSample: terminalPoint,
      },
      samples: [...samples, terminalPoint],
      closestPosition: terminalPoint.position,
    };
  }

  return {
    evaluation,
    samples: [...samples, terminalPoint],
    closestPosition: evaluation
      ? evaluation.hitPoint ?? evaluation.closestSample.position
      : terminalPoint.position,
  };
}

function evaluateCandidate(context, provider, yawRadians, pitchRadians) {
  const trajectory = index.normalizeTrajectorySource(
    provider.getTrajectory(context, {
      yawRadians,
      pitchRadians,
    }),
  );
  const { evaluation, samples, closestPosition } = evaluateWithTerminalPoint(
    context,
    trajectory,
  );

  if (!evaluation) {
    return null;
  }

  return {
    hit: evaluation.hit,
    qualityDistance: formatNumber(getAimQualityDistance(context, evaluation)),
    distance: formatNumber(evaluation.distance),
    closestPosition: formatPoint(closestPosition),
    closestTick: evaluation.closestSample.tick,
    travelState: getPitchTravelState(context, samples),
    sampleCount: samples.length,
  };
}

function runHeuristicProbe(input) {
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

  const history = [];
  const deltaMagnitude = roundPitchValue(Math.abs(input.initialDeltaRadians ?? 0.04));
  let currentPitch = input.pitchSeed;
  let currentDelta = 0;
  let currentDirection = 0;
  let currentResult = null;
  let firstHitResult = null;
  let seekingReturnHit = false;

  for (let iteration = 0; iteration < (input.maxIterations ?? 20); iteration += 1) {
    const evaluated = evaluateCandidate(
      context,
      provider,
      input.yawRadians,
      currentPitch,
    );

    if (!evaluated) {
      history.push({
        iteration,
        pitch: formatNumber(currentPitch),
        delta: formatNumber(currentDelta),
        outcome: "no-evaluation",
      });
      break;
    }

    let branch = "continue";

    if (firstHitResult) {
      if (!seekingReturnHit) {
        if (evaluated.hit) {
          branch = "inside-hit-band";
          currentResult = evaluated;
          currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
          currentPitch = roundPitchValue(currentPitch + currentDelta);
        } else {
          branch = "left-hit-band-reverse";
          seekingReturnHit = true;
          currentDirection *= -1;
          currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
          currentPitch = roundPitchValue(currentPitch + currentDelta);
        }
      } else if (evaluated.hit) {
        branch = "returned-hit-stop";
        history.push({
          iteration,
          pitch: formatNumber(currentPitch),
          delta: formatNumber(currentDelta),
          ...evaluated,
          branch,
        });
        break;
      } else {
        branch = "seek-return-hit";
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(currentPitch + currentDelta);
      }
    } else if (evaluated.hit) {
      branch = "first-hit";
      firstHitResult = evaluated;
      currentResult = evaluated;
      currentDirection = Math.sign(input.initialDeltaRadians ?? 0.04) || 1;
      currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
      currentPitch = roundPitchValue(currentPitch + currentDelta);
    } else if (!currentResult) {
      branch = "seed-miss";
      currentResult = evaluated;
      currentDirection = Math.sign(input.initialDeltaRadians ?? 0.04) || 1;
      currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
      currentPitch = roundPitchValue(currentPitch + currentDelta);
    } else {
      const changedDirection = evaluated.travelState !== currentResult.travelState;
      const improvedDistance = evaluated.qualityDistance < currentResult.qualityDistance;

      if (changedDirection) {
        branch = "travel-state-flip";
        currentResult = evaluated;
        currentDirection *= -1;
        currentDelta = roundPitchValue(deltaMagnitude * 0.5 * currentDirection);
        currentPitch = roundPitchValue(currentPitch + currentDelta);
      } else if (improvedDistance) {
        branch = "improved-keep-going";
        currentResult = evaluated;
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(currentPitch + currentDelta);
      } else {
        branch = "worse-reverse";
        currentDirection *= -1;
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(currentResult ? input.pitchSeed + 0 : currentPitch);
        currentPitch = roundPitchValue(
          (currentResult?.pitch ?? 0) + currentDelta,
        );
      }
    }

    history.push({
      iteration,
      pitch: formatNumber(currentPitch - currentDelta),
      delta: formatNumber(currentDelta),
      ...evaluated,
      branch,
    });
  }

  return history;
}

function firstDivergence(a, b) {
  const maxLength = Math.max(a.length, b.length);

  for (let i = 0; i < maxLength; i += 1) {
    const left = a[i] ?? null;
    const right = b[i] ?? null;

    if (JSON.stringify(left) !== JSON.stringify(right)) {
      return {
        iteration: i,
        withCollisions: left,
        withoutCollisions: right,
      };
    }
  }

  return null;
}

const scenario = {
  landingPos: new setup.Vec3(-79.5, 4.0, -29.5),
  yawRadians: 2.120331882934733,
  pitchSeed: -0.28372138980855915,
  initialDeltaRadians: 0.04,
  maxIterations: 20,
};

const withCollisions = runHeuristicProbe({
  ...scenario,
  blockChecking: true,
});
const withoutCollisions = runHeuristicProbe({
  ...scenario,
  blockChecking: false,
});

console.log(
  JSON.stringify(
    {
      firstDivergence: firstDivergence(withCollisions, withoutCollisions),
      withCollisions,
      withoutCollisions,
    },
    null,
    2,
  ),
);
