export { AABBUtils } from "@nxg-org/mineflayer-util-plugin";
export { Enderman, EnderShotFactory } from "@nxg-org/mineflayer-ender";
export { Vec3 } from "vec3";
import type { Bot } from "mineflayer";
import { Vec3 } from "vec3";

export type { Bot } from "mineflayer";

export interface Vec3Instance {
  readonly x: number;
  readonly y: number;
  readonly z: number;
  clone(): Vec3Instance;
  floored(): Vec3Instance;
  translate(x: number, y: number, z: number): Vec3Instance;
}

export interface ReplayBot {
  on(event: string, listener: (...args: unknown[]) => void): void;
  blockAt(pos: Vec3Instance): unknown;
  entity: {
    position: Vec3;
    yaw: number;
    pitch: number;
    velocity: Vec3;
    onGround: boolean;
  };
}

export interface ReplayBotOptions {
  readonly solidBlocks?: readonly Vec3Instance[];
  readonly includeGroundPlane?: boolean;
}

export function toVector3(vector: Vec3Instance) {
  return {
    x: vector.x,
    y: vector.y,
    z: vector.z,
  };
}

export function getDistance(
  a: { x: number; y: number; z: number },
  b: { x: number; y: number; z: number },
): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;

  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function createSolidBlock(position: Vec3Instance) {
  return {
    name: "grass",
    type: 2,
    boundingBox: "block",
    shapes: [[0, 0, 0, 1, 1, 1]],
    position,
  };
}

export function createReplayBot(options: ReplayBotOptions = {}): Bot {
  const {
    solidBlocks = [],
    includeGroundPlane = true,
  } = options;

  return {
    on: () => undefined,
    blockAt: (pos: Vec3Instance) => {
      const floored = pos.floored();

      if (
        solidBlocks.some(
          (block) =>
            block.x === floored.x &&
            block.y === floored.y &&
            block.z === floored.z,
        )
      ) {
        return createSolidBlock(floored);
      }

      if (!includeGroundPlane || floored.y !== 3) return null;

      return createSolidBlock(floored);
    },
    entity: {
      position: new Vec3(-73.844, 4.0, -32.224),
      yaw: 0,
      pitch: 0,
      velocity: new Vec3(-0.088, -0.078, -0.125),
      onGround: true,
    },
  } as unknown as Bot;
}
