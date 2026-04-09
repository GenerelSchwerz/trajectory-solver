import type {
  AimContext,
  LaunchCandidate,
  TrajectoryProvider,
  Vector3,
} from "../src/index";

export function nearlyEqual(a: number, b: number, epsilon = 1e-6): boolean {
  return Math.abs(a - b) <= epsilon;
}

export function createPointGeometry(target: Vector3) {
  return {
    contains(point: Vector3) {
      return (
        nearlyEqual(point.x, target.x) &&
        nearlyEqual(point.y, target.y) &&
        nearlyEqual(point.z, target.z)
      );
    },
  };
}

export function createDistanceToPoint(target: Vector3) {
  return (point: Vector3) =>
    Math.sqrt(
      (point.x - target.x) * (point.x - target.x) +
        (point.y - target.y) * (point.y - target.y) +
        (point.z - target.z) * (point.z - target.z),
    );
}

export function createContext(origin: Vector3, target: Vector3): AimContext {
  return {
    origin,
    target: {
      referencePoint: target,
      geometry: createPointGeometry(target),
    },
  };
}

export function createLinearProvider(): TrajectoryProvider {
  return {
    *getTrajectory(context, candidate) {
      for (let tick = 1; tick <= 10; tick += 1) {
        yield {
          tick,
          position: {
            x: context.origin.x + Math.sin(candidate.yawRadians) * tick,
            y: context.origin.y,
            z: context.origin.z + Math.cos(candidate.yawRadians) * tick,
          },
        };
      }
    },
  };
}

export function createCandidates(...candidates: LaunchCandidate[]) {
  return candidates;
}

export function createSegmentHitContext(): AimContext {
  return {
    origin: { x: 0, y: 0, z: 0 },
    target: {
      referencePoint: { x: 5, y: 0, z: 0 },
      geometry: {
        contains() {
          return false;
        },
        traceSegment(start, end) {
          if (start.x < 5 && end.x > 5) {
            return {
              hit: true,
              point: { x: 5, y: 0, z: 0 },
              face: "x=5",
            };
          }

          return null;
        },
        intersectsSegment(start, end) {
          return start.x < 5 && end.x > 5;
        },
        distanceTo(point) {
          return Math.abs(point.x - 5);
        },
      },
    },
  };
}

export function createSegmentHitProvider(): TrajectoryProvider {
  return {
    *getTrajectory(_context, candidate) {
      if (candidate.label === "segment-hit-x") {
        yield {
          tick: 1,
          position: { x: 4, y: 0, z: 0 },
        };
        yield {
          tick: 2,
          position: { x: 6, y: 0, z: 0 },
        };
        return;
      }

      yield {
        tick: 1,
        position: { x: 0, y: 0, z: 4 },
      };
      yield {
        tick: 2,
        position: { x: 0, y: 0, z: 6 },
      };
    },
  };
}

export function createPitchHeuristicContext(): AimContext {
  return {
    origin: { x: 0, y: 0, z: 0 },
    target: {
      referencePoint: { x: 10, y: 4, z: 0 },
      geometry: {
        contains(point) {
          return nearlyEqual(point.x, 10) && nearlyEqual(point.y, 4) && nearlyEqual(point.z, 0);
        },
        distanceTo(point) {
          return Math.abs(point.y - 4);
        },
      },
    },
  };
}

export function createPitchHeuristicProvider(): TrajectoryProvider {
  return {
    *getTrajectory(_context, candidate) {
      yield {
        tick: 1,
        position: {
          x: 10,
          y: candidate.pitchRadians * 10,
          z: 0,
        },
      };
    },
  };
}

export function createIdealPointHeuristicContext(): AimContext {
  return {
    origin: { x: 0, y: 0, z: 0 },
    target: {
      idealPoint: { x: 10, y: 4.2, z: 0 },
      referencePoint: { x: 10, y: 4, z: 0 },
      geometry: {
        idealPoint: { x: 10, y: 4.2, z: 0 },
        contains(point) {
          return nearlyEqual(point.x, 10) && point.y >= 3.5 && point.y <= 4.5 && nearlyEqual(point.z, 0);
        },
        distanceTo(point) {
          if (point.y >= 3.5 && point.y <= 4.5) {
            return 0;
          }

          if (point.y < 3.5) {
            return 3.5 - point.y;
          }

          return point.y - 4.5;
        },
      },
    },
  };
}
