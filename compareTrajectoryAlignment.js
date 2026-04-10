const setup = require("./test/mineflayer/setup.ts");

function getOriginVelocity(bot) {
  return bot.entity.velocity
    .clone()
    .translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
}

function formatVec(vec) {
  if (!vec) {
    return null;
  }

  return {
    x: Number(vec.x.toFixed(6)),
    y: Number(vec.y.toFixed(6)),
    z: Number(vec.z.toFixed(6)),
  };
}

function diffPoint(a, b) {
  if (!a || !b) {
    return null;
  }

  return {
    dx: Number((a.x - b.x).toFixed(9)),
    dy: Number((a.y - b.y).toFixed(9)),
    dz: Number((a.z - b.z).toFixed(9)),
  };
}

function compareShots(input) {
  const bot = setup.createReplayBot(input.botOptions);
  const landingAABB = setup.AABBUtils.getEntityAABBRaw({
    position: input.landingPos,
    height: 1.8,
    width: 0.6,
  });
  const originVelocity = getOriginVelocity(bot);

  function createShotResult(blockChecking) {
    const shot = setup.EnderShotFactory.fromPlayer(
      {
        position: bot.entity.position,
        yaw: input.yawRadians,
        pitch: input.pitchRadians,
        velocity: originVelocity,
      },
      bot,
    );
    const result = shot.calcToAABB(
      landingAABB,
      input.landingPos,
      blockChecking,
    );

    return {
      blockChecking,
      shot,
      result,
    };
  }

  const withBlocks = createShotResult(true);
  const withoutBlocks = createShotResult(false);
  const maxShared = Math.min(
    withBlocks.shot.points.length,
    withoutBlocks.shot.points.length,
  );
  let firstDifferentIndex = -1;

  for (let index = 0; index < maxShared; index += 1) {
    const a = withBlocks.shot.points[index];
    const b = withoutBlocks.shot.points[index];
    const equal =
      Math.abs(a.x - b.x) <= 1e-9 &&
      Math.abs(a.y - b.y) <= 1e-9 &&
      Math.abs(a.z - b.z) <= 1e-9;

    if (!equal) {
      firstDifferentIndex = index;
      break;
    }
  }

  const collisionStopIndex = withBlocks.shot.points.length;
  const prefixAligned =
    firstDifferentIndex === -1 || firstDifferentIndex >= collisionStopIndex;

  return {
    scenario: input.name,
    withBlocks: {
      pointCount: withBlocks.shot.points.length,
      closestPoint: formatVec(withBlocks.result.closestPoint),
      block: withBlocks.result.block
        ? {
            name: withBlocks.result.block.name,
            position: formatVec(withBlocks.result.block.position),
          }
        : null,
      totalTicks: withBlocks.result.totalTicks,
    },
    withoutBlocks: {
      pointCount: withoutBlocks.shot.points.length,
      closestPoint: formatVec(withoutBlocks.result.closestPoint),
      block: withoutBlocks.result.block
        ? {
            name: withoutBlocks.result.block.name,
            position: formatVec(withoutBlocks.result.block.position),
          }
        : null,
      totalTicks: withoutBlocks.result.totalTicks,
    },
    collisionStopIndex,
    firstDifferentIndex,
    prefixAlignedUntilCollision: prefixAligned,
    firstDifference:
      firstDifferentIndex >= 0
        ? {
            withBlocks: formatVec(withBlocks.shot.points[firstDifferentIndex]),
            withoutBlocks: formatVec(withoutBlocks.shot.points[firstDifferentIndex]),
            delta: diffPoint(
              withBlocks.shot.points[firstDifferentIndex],
              withoutBlocks.shot.points[firstDifferentIndex],
            ),
          }
        : null,
  };
}

const scenarios = [
  {
    name: "baseline-open-shot",
    landingPos: new setup.Vec3(-79.5, 4.0, -29.5),
    yawRadians: 2.120331882934733,
    pitchRadians: -0.20372138980855915,
  },
  {
    name: "blocked-shot",
    landingPos: new setup.Vec3(-79.0, 4.0, -39.5),
    yawRadians: 0.6168100829247134,
    pitchRadians: -0.12217304763960825,
    botOptions: {
      solidBlocks: [new setup.Vec3(-77, 4, -36), new setup.Vec3(-77, 5, -36)],
    },
  },
];

for (const scenario of scenarios) {
  console.log(JSON.stringify(compareShots(scenario), null, 2));
}
