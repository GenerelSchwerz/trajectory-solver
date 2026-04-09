
const setup = require('./test/mineflayer/setup.ts');
const utils = require('./test/mineflayer/enderTestUtils.ts');
const index = require('./src/index.ts');
const bot = setup.createReplayBot();
const enderman = new setup.Enderman(bot);
const landingPos = new setup.Vec3(-121.0, 4.0, -29.5);
const landingAABB = setup.AABBUtils.getEntityAABBRaw({ position: landingPos, height: 1.8, width: 0.6 });
const originVelocity = bot.entity.velocity.clone().translate(0, bot.entity.onGround ? -bot.entity.velocity.y : 0, 0);
const minFlightTicks = 30;
enderman.maxTicks = 140;
enderman.dvStep = 360;
const baseline = enderman.shotToAABB(landingAABB, landingPos, undefined, minFlightTicks);
const context = utils.createReplayContext(setup.toVector3(bot.entity.position), landingAABB, setup.toVector3(landingPos));
const provider = utils.createTrajectoryProvider(bot, landingAABB, landingPos, originVelocity);
const solution = index.solveAim({ context, provider, candidates: [{ yawRadians: baseline.yaw, pitchRadians: baseline.pitch }] });
const diag = utils.createShotDiagnostics(bot, landingAABB, landingPos, baseline.yaw, baseline.pitch, originVelocity);
console.log(JSON.stringify({ baseline: { hit: baseline.hit, pitch: baseline.pitch, ticks: baseline.ticks }, solveAimAtBaseline: solution && { hit: solution.hit, pitch: solution.pitchRadians, distance: solution.distance, closest: solution.closestPosition }, diag: { closestPoint: diag.result.closestPoint && setup.toVector3(diag.result.closestPoint), hit: diag.result.hit } }, null, 2));