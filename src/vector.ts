export interface Vector3 {
  readonly x: number;
  readonly y: number;
  readonly z: number;
}

export function addVectors(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.x + b.x,
    y: a.y + b.y,
    z: a.z + b.z,
  };
}

export function subtractVectors(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.x - b.x,
    y: a.y - b.y,
    z: a.z - b.z,
  };
}

export function scaleVector(vector: Vector3, scalar: number): Vector3 {
  return {
    x: vector.x * scalar,
    y: vector.y * scalar,
    z: vector.z * scalar,
  };
}

export function dotVectors(a: Vector3, b: Vector3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

export function getMagnitude(vector: Vector3): number {
  return Math.sqrt(dotVectors(vector, vector));
}

export function getDistance(a: Vector3, b: Vector3): number {
  return getMagnitude(subtractVectors(a, b));
}

export function normalizeVector(vector: Vector3): Vector3 {
  const magnitude = getMagnitude(vector);

  if (magnitude === 0) {
    return { x: 0, y: 0, z: 0 };
  }

  return scaleVector(vector, 1 / magnitude);
}

export function vectorFromYawPitch(yawRadians: number, pitchRadians: number): Vector3 {
  const horizontalMagnitude = Math.cos(pitchRadians);

  return normalizeVector({
    x: Math.sin(yawRadians) * horizontalMagnitude,
    y: Math.sin(pitchRadians),
    z: Math.cos(yawRadians) * horizontalMagnitude,
  });
}

export function yawPitchFromDirection(direction: Vector3): {
  yawRadians: number;
  pitchRadians: number;
} {
  const normalized = normalizeVector(direction);

  return {
    yawRadians: Math.atan2(normalized.x, normalized.z),
    pitchRadians: Math.atan2(
      normalized.y,
      Math.hypot(normalized.x, normalized.z),
    ),
  };
}
