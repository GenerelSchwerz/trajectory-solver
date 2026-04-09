import type {
  AimContext,
  AimEvaluation,
  AimSearchRequest,
  AimSolution,
  CandidateGenerator,
  EvaluatedTrajectory,
  LaunchCandidate,
  PitchHeuristicIteration,
  PitchHeuristicSearchRequest,
  TrajectoryComputation,
  TrajectoryInfo,
  TrajectoryProvider,
  TrajectorySample,
  TrajectorySource,
} from "./types";
import {
  dotVectors,
  getDistance,
  normalizeVector,
  subtractVectors,
  vectorFromYawPitch,
  type Vector3,
  yawPitchFromDirection,
} from "./vector";

function getIdealPoint(context: AimContext): Vector3 | null {
  return context.target.idealPoint ?? context.target.geometry.idealPoint ?? null;
}

function getReferencePoint(context: AimContext): Vector3 {
  return (
    context.target.referencePoint ??
    context.target.geometry.referencePoint ??
    getIdealPoint(context) ??
    context.origin
  );
}

function compareEvaluations(
  candidateEvaluation: AimEvaluation,
  bestEvaluation: AimEvaluation | null,
): boolean {
  if (!bestEvaluation) {
    return true;
  }

  if (candidateEvaluation.hit && !bestEvaluation.hit) {
    return true;
  }

  return (
    candidateEvaluation.hit === bestEvaluation.hit &&
    candidateEvaluation.distance < bestEvaluation.distance
  );
}

function targetContainsPoint(
  context: AimContext,
  point: TrajectorySample["position"],
): boolean {
  const { geometry } = context.target;

  if (geometry.isHit) {
    return geometry.isHit(point);
  }

  if (geometry.contains) {
    return geometry.contains(point);
  }

  if (geometry.intersects) {
    return geometry.intersects(point);
  }

  return false;
}

function targetIntersectsSegment(
  context: AimContext,
  start: Vector3,
  end: Vector3,
): boolean {
  return context.target.geometry.intersectsSegment?.(start, end) ?? false;
}

function traceTargetSegment(
  context: AimContext,
  start: Vector3,
  end: Vector3,
) {
  return context.target.geometry.traceSegment?.(start, end) ?? null;
}

function getTargetDistance(context: AimContext, sample: TrajectorySample): number {
  if (context.target.geometry.distanceTo) {
    return context.target.geometry.distanceTo(sample.position);
  }

  return getDistance(sample.position, getIdealPoint(context) ?? getReferencePoint(context));
}

function isTrajectoryComputation<TInfo extends TrajectoryInfo>(
  source: TrajectorySource<TInfo>,
): source is TrajectoryComputation<TInfo> {
  return "samples" in source && "info" in source;
}

export function normalizeTrajectorySource<TInfo extends TrajectoryInfo>(
  source: TrajectorySource<TInfo>,
): EvaluatedTrajectory<TInfo> {
  if (isTrajectoryComputation(source)) {
    return {
      samples: source.samples,
      info: source.info,
    };
  }

  return {
    samples: source,
    info: {
      kind: "piecewise",
    } as TInfo,
  };
}

export function evaluateTrajectory(
  context: AimContext,
  samples: Iterable<TrajectorySample>,
): AimEvaluation | null {
  let closestSample: TrajectorySample | null = null;
  let shortestDistance = Number.POSITIVE_INFINITY;
  let previousSample: TrajectorySample | null = null;

  for (const sample of samples) {
    const distance = getTargetDistance(context, sample);

    if (distance < shortestDistance) {
      shortestDistance = distance;
      closestSample = sample;
    }

    if (targetContainsPoint(context, sample.position)) {
      return {
        hit: true,
        distance,
        closestSample: sample,
        hitPoint: sample.position,
      };
    }

    if (previousSample) {
      const tracedSegmentHit = traceTargetSegment(
        context,
        previousSample.position,
        sample.position,
      );

      if (tracedSegmentHit?.hit) {
        return {
          hit: true,
          distance: tracedSegmentHit.point
            ? context.target.geometry.distanceTo?.(tracedSegmentHit.point) ??
              getDistance(tracedSegmentHit.point, getReferencePoint(context))
            : distance,
          closestSample: sample,
          hitPoint: tracedSegmentHit.point ?? sample.position,
          hitFace: tracedSegmentHit.face,
        };
      }

      if (
        targetIntersectsSegment(
          context,
          previousSample.position,
          sample.position,
        )
      ) {
        return {
          hit: true,
          distance,
          closestSample: sample,
          hitPoint: sample.position,
        };
      }
    }

    previousSample = sample;
  }

  if (!closestSample) {
    return null;
  }

  return {
    hit: false,
    distance: shortestDistance,
    closestSample,
  };
}

export function createAimSolution<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  context: TContext,
  candidate: TCandidate,
  evaluation: AimEvaluation,
  trajectory: EvaluatedTrajectory<TInfo>,
): AimSolution<TContext, TCandidate, TInfo> {
  const candidateDirection = vectorFromYawPitch(
    candidate.yawRadians,
    candidate.pitchRadians,
  );
  const fallbackDirection = normalizeVector(
    subtractVectors(evaluation.closestSample.position, context.origin),
  );
  const direction =
    candidateDirection.x === 0 &&
    candidateDirection.y === 0 &&
    candidateDirection.z === 0
      ? fallbackDirection
      : candidateDirection;

  return {
    hit: evaluation.hit,
    yawRadians: candidate.yawRadians,
    pitchRadians: candidate.pitchRadians,
    direction,
    distance: evaluation.distance,
    closestPosition: evaluation.hitPoint ?? evaluation.closestSample.position,
    tick: evaluation.closestSample.tick,
    candidate,
    context,
    trajectory,
  };
}

function evaluateCandidate<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  context: TContext,
  provider: TrajectoryProvider<TContext, TCandidate, TInfo>,
  candidate: TCandidate,
): {
  candidate: TCandidate;
  evaluation: AimEvaluation;
  trajectory: EvaluatedTrajectory<TInfo>;
  solution: AimSolution<TContext, TCandidate, TInfo>;
} | null {
  const trajectory = normalizeTrajectorySource(
    provider.getTrajectory(context, candidate),
  );
  const evaluation = evaluateTrajectory(context, trajectory.samples);

  if (!evaluation) {
    return null;
  }

  return {
    candidate,
    evaluation,
    trajectory,
    solution: createAimSolution(context, candidate, evaluation, trajectory),
  };
}

function getAimQualityDistance(
  context: AimContext,
  evaluation: AimEvaluation,
): number {
  const idealPoint = getIdealPoint(context);

  if (!idealPoint || !evaluation.hit) {
    return evaluation.distance;
  }

  return getDistance(
    evaluation.hitPoint ?? evaluation.closestSample.position,
    idealPoint,
  );
}

function getClosestSampleDirection(
  samples: readonly TrajectorySample[],
  closestSample: TrajectorySample,
): Vector3 {
  const closestIndex = samples.findIndex((sample) => sample === closestSample);
  const currentSample = closestIndex >= 0 ? samples[closestIndex] : closestSample;

  if (currentSample.velocity) {
    return currentSample.velocity;
  }

  const previousSample = closestIndex > 0 ? samples[closestIndex - 1] : null;
  const nextSample =
    closestIndex >= 0 && closestIndex < samples.length - 1
      ? samples[closestIndex + 1]
      : null;

  if (previousSample && nextSample) {
    return subtractVectors(nextSample.position, previousSample.position);
  }

  if (nextSample) {
    return subtractVectors(nextSample.position, currentSample.position);
  }

  if (previousSample) {
    return subtractVectors(currentSample.position, previousSample.position);
  }

  return { x: 0, y: 0, z: 0 };
}

function getPitchTravelState(
  context: AimContext,
  evaluation: AimEvaluation,
  samples: readonly TrajectorySample[],
): "overshoot" | "undershoot" {
  const targetPoint = getIdealPoint(context) ?? getReferencePoint(context);
  const closestPoint = evaluation.hitPoint ?? evaluation.closestSample.position;
  const velocity = getClosestSampleDirection(samples, evaluation.closestSample);
  const toTarget = subtractVectors(targetPoint, closestPoint);
  const direction =
    velocity.x === 0 && velocity.y === 0 && velocity.z === 0
      ? subtractVectors(closestPoint, context.origin)
      : velocity;

  return dotVectors(direction, toTarget) >= 0 ? "undershoot" : "overshoot";
}

function roundPitchValue(value: number): number {
  return Number(value.toFixed(12));
}

type PitchHeuristicResult<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
> = {
  candidate: TCandidate;
  evaluation: AimEvaluation;
  trajectory: EvaluatedTrajectory<TInfo>;
  solution: AimSolution<TContext, TCandidate, TInfo>;
  qualityDistance: number;
  travelState: "overshoot" | "undershoot";
};

function materializeHeuristicResult<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  context: TContext,
  result: {
    candidate: TCandidate;
    evaluation: AimEvaluation;
    trajectory: EvaluatedTrajectory<TInfo>;
    solution: AimSolution<TContext, TCandidate, TInfo>;
  },
): PitchHeuristicResult<TContext, TCandidate, TInfo> {
  const samples = Array.isArray(result.trajectory.samples)
    ? result.trajectory.samples
    : Array.from(result.trajectory.samples);
  const trajectory = {
    ...result.trajectory,
    samples,
  };

  return {
    ...result,
    trajectory,
    solution: createAimSolution(
      context,
      result.candidate,
      result.evaluation,
      trajectory,
    ),
    qualityDistance: getAimQualityDistance(context, result.evaluation),
    travelState: getPitchTravelState(
      context,
      result.evaluation,
      samples,
    ),
  };
}

function updateBestHeuristicResult<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  bestResult: PitchHeuristicResult<TContext, TCandidate, TInfo> | null,
  candidateResult: PitchHeuristicResult<TContext, TCandidate, TInfo>,
  bestQualityDistance: number,
) {
  if (
    !bestResult ||
    (candidateResult.evaluation.hit && !bestResult.evaluation.hit) ||
    (candidateResult.evaluation.hit === bestResult.evaluation.hit &&
      candidateResult.qualityDistance < bestQualityDistance)
  ) {
    return {
      bestResult: candidateResult,
      bestQualityDistance: candidateResult.qualityDistance,
    };
  }

  return {
    bestResult,
    bestQualityDistance,
  };
}

function averagePoints(a: Vector3, b: Vector3): Vector3 {
  return {
    x: (a.x + b.x) / 2,
    y: (a.y + b.y) / 2,
    z: (a.z + b.z) / 2,
  };
}

function createDefaultPitchHeuristicCandidate<
  TCandidate extends LaunchCandidate,
>(
  input: {
    baseCandidate: TCandidate;
    pitchRadians: number;
    deltaRadians: number;
    iteration: number;
    history: readonly PitchHeuristicIteration[];
  },
): TCandidate {
  const candidate: LaunchCandidate = {
    ...input.baseCandidate,
    pitchRadians: input.pitchRadians,
    metadata: {
      ...input.baseCandidate.metadata,
      pitchHeuristic: {
        iteration: input.iteration,
        deltaRadians: input.deltaRadians,
        history: [...input.history],
      },
    },
  };

  return candidate as TCandidate;
}

export function solveAim<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  request: AimSearchRequest<TContext, TCandidate, TInfo>,
): AimSolution<TContext, TCandidate, TInfo> | null {
  let bestCandidate: TCandidate | null = null;
  let bestEvaluation: AimEvaluation | null = null;
  let bestTrajectory: EvaluatedTrajectory<TInfo> | null = null;

  for (const candidate of request.candidates) {
    const result = evaluateCandidate(
      request.context,
      request.provider,
      candidate,
    );

    if (!result) {
      continue;
    }

    if (compareEvaluations(result.evaluation, bestEvaluation)) {
      bestCandidate = result.candidate;
      bestEvaluation = result.evaluation;
      bestTrajectory = result.trajectory;
    }
  }

  if (!bestCandidate || !bestEvaluation || !bestTrajectory) {
    return null;
  }

  return createAimSolution(
    request.context,
    bestCandidate,
    bestEvaluation,
    bestTrajectory,
  );
}

export function solveAimWithPitchHeuristic<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  request: PitchHeuristicSearchRequest<TContext, TCandidate, TInfo>,
): AimSolution<TContext, TCandidate, TInfo> | null {
  const maxIterations = request.maxIterations ?? 12;
  const createCandidate =
    request.createCandidate ?? createDefaultPitchHeuristicCandidate<TCandidate>;
  let bestResult:
    | PitchHeuristicResult<TContext, TCandidate, TInfo>
    | null = null;
  let bestQualityDistance = Number.POSITIVE_INFINITY;
  let deltaMagnitude = roundPitchValue(Math.abs(request.initialDeltaRadians));

  for (const baseCandidate of request.candidates) {
    const history: PitchHeuristicIteration[] = [];
    let currentPitch = baseCandidate.pitchRadians;
    let currentDelta = 0;
    let currentDirection = 0;
    let currentResult:
      | PitchHeuristicResult<TContext, TCandidate, TInfo>
      | null = null;
    let firstHitResult:
      | PitchHeuristicResult<TContext, TCandidate, TInfo>
      | null = null;
    const hitBandResults: PitchHeuristicResult<TContext, TCandidate, TInfo>[] = [];
    let seekingReturnHit = false;

    for (let iteration = 0; iteration < maxIterations; iteration += 1) {
      const candidate = createCandidate({
        baseCandidate,
        pitchRadians: currentPitch,
        deltaRadians: currentDelta,
        iteration,
        history,
      });
      const result = evaluateCandidate(
        request.context,
        request.provider,
        candidate,
      );

      if (!result) {
        break;
      }

      const evaluatedResult = materializeHeuristicResult(request.context, result);

      history.push({
        iteration,
        pitchRadians: roundPitchValue(currentPitch),
        deltaRadians: roundPitchValue(currentDelta),
        distance: evaluatedResult.qualityDistance,
        hit: result.evaluation.hit,
      });

      ({
        bestResult,
        bestQualityDistance,
      } = updateBestHeuristicResult(
        bestResult,
        evaluatedResult,
        bestQualityDistance,
      ));

      if (firstHitResult) {
        if (!seekingReturnHit) {
          if (evaluatedResult.evaluation.hit) {
            hitBandResults.push(evaluatedResult);
            currentResult = evaluatedResult;
            currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
            currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
            continue;
          }

          seekingReturnHit = true;
          currentDirection *= -1;
          currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
          currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
          continue;
        }

        if (evaluatedResult.evaluation.hit) {
          hitBandResults.push(evaluatedResult);
          const targetPoint = averagePoints(
            firstHitResult.solution.closestPosition,
            evaluatedResult.solution.closestPosition,
          );
          const centeredHit = hitBandResults.reduce((best, candidateResult) =>
            getDistance(candidateResult.solution.closestPosition, targetPoint) <
            getDistance(best.solution.closestPosition, targetPoint)
              ? candidateResult
              : best,
          );
          bestResult = centeredHit;
          bestQualityDistance = centeredHit.qualityDistance;

          break;
        }

        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
        continue;
      }

      if (iteration === maxIterations - 1) {
        break;
      }

      if (evaluatedResult.evaluation.hit) {
        firstHitResult = evaluatedResult;
        hitBandResults.push(evaluatedResult);
        currentResult = evaluatedResult;
        currentDirection =
          request.initialDeltaRadians === 0
            ? 1
            : Math.sign(request.initialDeltaRadians);
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
        continue;
      }

      if (!currentResult) {
        currentResult = evaluatedResult;
        currentDirection =
          request.initialDeltaRadians === 0
            ? 1
            : Math.sign(request.initialDeltaRadians);
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(currentPitch + currentDelta);
        continue;
      }

      const changedDirection =
        evaluatedResult.travelState !== currentResult.travelState;
      const improvedDistance =
        evaluatedResult.qualityDistance < currentResult.qualityDistance;

      if (changedDirection) {
        currentResult = evaluatedResult;
        currentDirection *= -1;
        currentDelta = roundPitchValue(deltaMagnitude * 0.5 * currentDirection);
        currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
        continue;
      }

      if (improvedDistance) {
        currentResult = evaluatedResult;
        currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
        currentPitch = roundPitchValue(evaluatedResult.candidate.pitchRadians + currentDelta);
        continue;
      }

      currentDirection *= -1;
      currentDelta = roundPitchValue(deltaMagnitude * currentDirection);
      currentPitch = roundPitchValue(currentResult.candidate.pitchRadians + currentDelta);
    }
  }

  return bestResult?.solution ?? null;
}

export function solveAimWithGenerator<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  context: TContext,
  provider: TrajectoryProvider<TContext, TCandidate, TInfo>,
  generator: CandidateGenerator<TContext, TCandidate>,
): AimSolution<TContext, TCandidate, TInfo> | null {
  return solveAim({
    context,
    provider,
    candidates: generator.getCandidates(context),
  });
}

export function createDirectAimSolution(context: AimContext): AimSolution<AimContext> {
  const referencePoint = getReferencePoint(context);
  const direction = normalizeVector(
    subtractVectors(referencePoint, context.origin),
  );
  const angles = yawPitchFromDirection(direction);

  return {
    hit: true,
    yawRadians: angles.yawRadians,
    pitchRadians: angles.pitchRadians,
    direction,
    distance: getDistance(context.origin, referencePoint),
    closestPosition: referencePoint,
    tick: 0,
    candidate: {
      yawRadians: angles.yawRadians,
      pitchRadians: angles.pitchRadians,
    },
    context,
    trajectory: {
      samples: [
        {
          tick: 0,
          position: referencePoint,
        },
      ],
      info: {
        kind: "formulaic",
        equation: "direct-line",
        closedForm: true,
        solvedFor: "aim-direction",
      },
    },
  };
}
