import { performance } from "node:perf_hooks";

import {
  GridCandidateGenerator,
  solveAim,
  solveAimWithPitchHeuristic,
  type LaunchCandidate,
  type TrajectoryProvider,
} from "../../src/index";
import {
  AABBUtils,
  Bot,
  EnderShotFactory,
  getDistance,
  toVector3,
  Vec3,
  type ReplayBot,
} from "./setup";

export type Point3 = { x: number; y: number; z: number };
export type ReplayAabb = ReturnType<typeof AABBUtils.getEntityAABBRaw>;
export type ReplaySolution = ReturnType<typeof solveAim>;

function getMagnitude(vector: Point3): number {
  return Math.sqrt(
    vector.x * vector.x +
      vector.y * vector.y +
      vector.z * vector.z,
  );
}

function getAabbFace(
  point: Point3,
  bounds: {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
  },
): string | undefined {
  const epsilon = 1e-6;

  if (Math.abs(point.x - bounds.minX) <= epsilon) return "minX";
  if (Math.abs(point.x - bounds.maxX) <= epsilon) return "maxX";
  if (Math.abs(point.y - bounds.minY) <= epsilon) return "minY";
  if (Math.abs(point.y - bounds.maxY) <= epsilon) return "maxY";
  if (Math.abs(point.z - bounds.minZ) <= epsilon) return "minZ";
  if (Math.abs(point.z - bounds.maxZ) <= epsilon) return "maxZ";

  return undefined;
}

export function createTargetGeometry(landingAABB: ReplayAabb) {
  return {
    contains(point: Point3) {
      return !!landingAABB.containsVec(new Vec3(point.x, point.y, point.z));
    },
    traceSegment(start: Point3, end: Point3) {
      const intersection = landingAABB.intersectsSegment(
        new Vec3(start.x, start.y, start.z),
        new Vec3(end.x, end.y, end.z),
      );

      if (!intersection) {
        return null;
      }

      const point = toVector3(intersection);

      return {
        hit: true,
        point,
        face: getAabbFace(point, {
          minX: landingAABB.minX,
          maxX: landingAABB.maxX,
          minY: landingAABB.minY,
          maxY: landingAABB.maxY,
          minZ: landingAABB.minZ,
          maxZ: landingAABB.maxZ,
        }),
      };
    },
    distanceTo(point: Point3) {
      return landingAABB.distanceToVec(new Vec3(point.x, point.y, point.z));
    },
  };
}

export function createReplayContext(
  origin: Point3,
  landingAABB: ReplayAabb,
  landingPos: Point3,
) {
  return {
    origin,
    target: {
      idealPoint: landingPos,
      referencePoint: landingPos,
      geometry: {
        ...createTargetGeometry(landingAABB),
        idealPoint: landingPos,
      },
    },
  };
}

export function estimatePitchWithLinearDrag(
  bot: Bot,
  yawRadians: number,
  target: Point3,
  originVelocity: Vec3,
): number {
  const probeShot = EnderShotFactory.fromPlayer(
    {
      position: bot.entity.position,
      yaw: yawRadians,
      pitch: 0,
      velocity: originVelocity,
    },
    bot,
  );

  const launchOrigin = toVector3(probeShot.initialPos);
  const launchSpeed = getMagnitude({
    x: probeShot.initialVel.x - originVelocity.x,
    y: probeShot.initialVel.y - originVelocity.y,
    z: probeShot.initialVel.z - originVelocity.z,
  });
  const gravity = probeShot.gravity;
  const drag = 0.01;
  const horizontalDistance = Math.hypot(
    target.x - launchOrigin.x,
    target.z - launchOrigin.z,
  );
  const verticalOffset = target.y - launchOrigin.y;
  const directPitch = Math.atan2(verticalOffset, horizontalDistance);
  const effectiveSpeed = Math.max(
    launchSpeed * 0.75,
    launchSpeed * (1 - (drag * horizontalDistance) / Math.max(launchSpeed, 1e-6)),
  );
  const speedSquared = effectiveSpeed * effectiveSpeed;
  const discriminant =
    speedSquared * speedSquared -
    gravity * (gravity * horizontalDistance * horizontalDistance + 2 * verticalOffset * speedSquared);

  if (discriminant < 0) {
    return directPitch;
  }

  const tangent =
    (speedSquared - Math.sqrt(discriminant)) /
    (gravity * Math.max(horizontalDistance, 1e-6));

  return Math.atan(tangent);
}

export function createPiecewisePoints(
  geometry: ReplayAabb,
  points: Point3[],
  closestPoint?: Point3 | null,
) {
  const allPoints = [
    ...points,
    ...(closestPoint ? [closestPoint] : []),
  ];

  return allPoints.map((point, index) => {
    const current = new Vec3(point.x, point.y, point.z);
    const previous = index > 0 ? allPoints[index - 1] : null;
    const segmentIntersected = previous
      ? !!geometry.intersectsSegment(
          new Vec3(previous.x, previous.y, previous.z),
          current,
        )
      : false;

    return {
      tick: index,
      x: point.x,
      y: point.y,
      z: point.z,
      segmentIntersected,
      distanceToGeometry: geometry.distanceToVec(current),
    };
  });
}

export function createTrajectoryProvider(
  bot: Bot,
  landingAABB: ReplayAabb,
  landingPos: Vec3,
  originVelocity: Vec3,
): TrajectoryProvider {
  return {
    *getTrajectory(_context, candidate) {
      const shot = EnderShotFactory.fromPlayer(
        {
          position: bot.entity.position,
          yaw: candidate.yawRadians,
          pitch: candidate.pitchRadians,
          velocity: originVelocity,
        },
        bot,
      );
      const result = shot.calcToAABB(landingAABB, landingPos, true);

      for (let index = 0; index < shot.points.length; index += 1) {
        yield {
          tick: index,
          position: toVector3(shot.points[index]),
        };
      }

      if (result.closestPoint) {
        yield {
          tick: shot.points.length,
          position: toVector3(result.closestPoint),
        };
      }
    },
  };
}

export function createCandidateGenerator(
  pitchSeed: number,
  yaw:
    | { fixedYaw: number; provideYaw?: () => number }
    | { provideYaw: () => number }
    | { yawSearch: { start: number; end: number; step: number }; provideYaw?: () => number },
) {
  return new GridCandidateGenerator(
    {
      pitch: { start: pitchSeed, end: pitchSeed, step: 0.01 },
      yawMode: "relative",
      ...yaw,
    },
  );
}

export function solveReplayAim(
  bot: Bot,
  landingAABB: ReplayAabb,
  landingPos: Vec3,
  originVelocity: Vec3,
  candidates: Iterable<LaunchCandidate>,
): { solution: ReplaySolution; elapsedMs: number } {
  const startedAt = performance.now();
  const solution = solveAim({
    context: createReplayContext(
      toVector3(bot.entity.position),
      landingAABB,
      toVector3(landingPos),
    ),
    provider: createTrajectoryProvider(bot, landingAABB, landingPos, originVelocity),
    candidates,
  });

  return {
    solution,
    elapsedMs: performance.now() - startedAt,
  };
}

export function solveReplayAimWithPitchHeuristic(
  bot: Bot,
  landingAABB: ReplayAabb,
  landingPos: Vec3,
  originVelocity: Vec3,
  candidates: Iterable<LaunchCandidate>,
  initialDeltaRadians = 0.04,
): { solution: ReplaySolution; elapsedMs: number } {
  const startedAt = performance.now();
  const solution = solveAimWithPitchHeuristic({
    context: createReplayContext(
      toVector3(bot.entity.position),
      landingAABB,
      toVector3(landingPos),
    ),
    provider: createTrajectoryProvider(bot, landingAABB, landingPos, originVelocity),
    candidates,
    initialDeltaRadians,
    maxIterations: 30,
  });

  return {
    solution,
    elapsedMs: performance.now() - startedAt,
  };
}

export function createShotDiagnostics(
  bot: Bot,
  landingAABB: ReplayAabb,
  landingPos: Vec3,
  yaw: number,
  pitch: number,
  originVelocity: Vec3,
) {
  const shot = EnderShotFactory.fromPlayer(
    {
      position: bot.entity.position,
      yaw,
      pitch,
      velocity: originVelocity,
    },
    bot,
  );
  const result = shot.calcToAABB(landingAABB, landingPos, true);

  return {
    shot,
    result,
    points: createPiecewisePoints(
      landingAABB,
      shot.points.map(toVector3),
      result.closestPoint ? toVector3(result.closestPoint) : null,
    ),
  };
}

export function logReplayValues(input: {
  enabled?: boolean;
  baselineElapsedMs: number;
  providedYawElapsedMs: number;
  summary: unknown;
}) {
  if (!input.enabled) {
    return;
  }

  console.log("mineflayer-ender solve time (ms):", input.baselineElapsedMs.toFixed(3));
  console.log("projectile-aim provided yaw solve time (ms):", input.providedYawElapsedMs.toFixed(3));
  console.log(
    "provided yaw check:\n" +
      JSON.stringify(input.summary, null, 2),
  );
}

export function getClosestPointDistance(a: Point3, b: Point3) {
  return getDistance(a, b);
}
