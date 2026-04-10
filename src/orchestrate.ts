import {
  solveAim,
  solveAimWithPitchHeuristic,
} from "./evaluate";
import type {
  AimContext,
  AimSolution,
  CollisionFallbackPitchSearchRequest,
  LaunchCandidate,
  PitchHeuristicIteration,
  TrajectoryInfo,
} from "./types";

type CollisionFallbackStage = {
  kind: "initial" | "alternate";
  seedPitchRadians: number;
  collisionFreeIterations: number;
  collisionFreeHit: boolean;
  collisionAwareHit: boolean;
};

type CollisionFallbackMetrics = {
  collisionFreeIterations: number;
  collisionAwareEvaluations: number;
  attemptedCandidates: number;
  stages: CollisionFallbackStage[];
};

function getPitchHeuristicIterations(candidate: LaunchCandidate | null | undefined): number {
  return (
    (
      candidate?.metadata?.pitchHeuristic as
        | { history?: PitchHeuristicIteration[] }
        | undefined
    )?.history?.length ?? 0
  );
}

function mergeCollisionFallbackMetrics(
  left: CollisionFallbackMetrics,
  right: CollisionFallbackMetrics,
): CollisionFallbackMetrics {
  return {
    collisionFreeIterations:
      left.collisionFreeIterations + right.collisionFreeIterations,
    collisionAwareEvaluations:
      left.collisionAwareEvaluations + right.collisionAwareEvaluations,
    attemptedCandidates: left.attemptedCandidates + right.attemptedCandidates,
    stages: [...left.stages, ...right.stages],
  };
}

function attachCollisionFallbackMetrics<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
>(
  solution: AimSolution<TContext, TCandidate, TInfo>,
  metrics: CollisionFallbackMetrics,
): AimSolution<TContext, TCandidate, TInfo> {
  const candidate = {
    ...solution.candidate,
    metadata: {
      ...solution.candidate.metadata,
      collisionFallbackPitchHeuristic: metrics,
    },
  } as TCandidate;

  return {
    ...solution,
    candidate,
  };
}

function validateCandidateWithCollisions<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TCollisionAwareInfo extends TrajectoryInfo,
>(
  request: {
    context: TContext;
    candidate: TCandidate;
    collisionAwareProvider: CollisionFallbackPitchSearchRequest<
      TContext,
      TCandidate,
      TrajectoryInfo,
      TCollisionAwareInfo
    >["collisionAwareProvider"];
  },
): AimSolution<TContext, TCandidate, TCollisionAwareInfo> | null {
  return solveAim({
    context: request.context,
    provider: request.collisionAwareProvider,
    candidates: [request.candidate],
  });
}

function solveBatchWithoutCollisions<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TCollisionFreeInfo extends TrajectoryInfo,
  TCollisionAwareInfo extends TrajectoryInfo,
>(
  request: CollisionFallbackPitchSearchRequest<
    TContext,
    TCandidate,
    TCollisionFreeInfo,
    TCollisionAwareInfo
  >,
  candidates: Iterable<TCandidate>,
  stageKind: "initial" | "alternate",
): {
  solution: AimSolution<TContext, TCandidate, TCollisionAwareInfo> | null;
  metrics: CollisionFallbackMetrics;
} {
  let metrics: CollisionFallbackMetrics = {
    collisionFreeIterations: 0,
    collisionAwareEvaluations: 0,
    attemptedCandidates: 0,
    stages: [],
  };

  for (const candidate of candidates) {
    metrics = {
      ...metrics,
      attemptedCandidates: metrics.attemptedCandidates + 1,
    };
    const collisionFreeSolution = solveAimWithPitchHeuristic({
      context: request.context,
      provider: request.collisionFreeProvider,
      candidates: [candidate],
      initialDeltaRadians: request.initialDeltaRadians,
      maxIterations: request.maxIterations,
      createCandidate: request.createCandidate,
    });

    if (!collisionFreeSolution) {
      metrics = {
        ...metrics,
        stages: [
          ...metrics.stages,
          {
            kind: stageKind,
            seedPitchRadians: candidate.pitchRadians,
            collisionFreeIterations: 0,
            collisionFreeHit: false,
            collisionAwareHit: false,
          },
        ],
      };
      continue;
    }

    const collisionFreeIterations = getPitchHeuristicIterations(
      collisionFreeSolution.candidate,
    );
    metrics = {
      ...metrics,
      collisionFreeIterations:
        metrics.collisionFreeIterations + collisionFreeIterations,
      collisionAwareEvaluations: metrics.collisionAwareEvaluations + 1,
    };

    const collisionAwareSolution = validateCandidateWithCollisions({
      context: request.context,
      candidate: collisionFreeSolution.candidate,
      collisionAwareProvider: request.collisionAwareProvider,
    });
    const stage: CollisionFallbackStage = {
      kind: stageKind,
      seedPitchRadians: candidate.pitchRadians,
      collisionFreeIterations,
      collisionFreeHit: collisionFreeSolution.hit,
      collisionAwareHit: collisionAwareSolution?.hit ?? false,
    };
    metrics = {
      ...metrics,
      stages: [...metrics.stages, stage],
    };

    if (collisionAwareSolution?.hit) {
      return {
        solution: attachCollisionFallbackMetrics(collisionAwareSolution, metrics),
        metrics,
      };
    }
  }

  return {
    solution: null,
    metrics,
  };
}

function solveBatchWithCollisions<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TCollisionFreeInfo extends TrajectoryInfo,
  TCollisionAwareInfo extends TrajectoryInfo,
>(
  request: CollisionFallbackPitchSearchRequest<
    TContext,
    TCandidate,
    TCollisionFreeInfo,
    TCollisionAwareInfo
  >,
  candidates: Iterable<TCandidate>,
): AimSolution<TContext, TCandidate, TCollisionAwareInfo> | null {
  return solveAimWithPitchHeuristic({
    context: request.context,
    provider: request.collisionAwareProvider,
    candidates,
    initialDeltaRadians: request.initialDeltaRadians,
    maxIterations: request.maxIterations,
    createCandidate: request.createCandidate,
  });
}

export function solveAimWithCollisionFallbackPitchHeuristic<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TCollisionFreeInfo extends TrajectoryInfo,
  TCollisionAwareInfo extends TrajectoryInfo,
>(
  request: CollisionFallbackPitchSearchRequest<
    TContext,
    TCandidate,
    TCollisionFreeInfo,
    TCollisionAwareInfo
  >,
): AimSolution<TContext, TCandidate, TCollisionAwareInfo> | null {
  const initialBatch = solveBatchWithoutCollisions(
    request,
    request.initialCandidates,
    "initial",
  );

  if (initialBatch.solution && initialBatch.solution.hit) {
    return initialBatch.solution;
  }

  if (!request.alternateCandidates) {
    return null;
  }

  const alternateBatch = solveBatchWithoutCollisions(
    request,
    request.alternateCandidates,
    "alternate",
  );

  if (!alternateBatch.solution) {
    return null;
  }

  return attachCollisionFallbackMetrics(
    alternateBatch.solution,
    mergeCollisionFallbackMetrics(
      initialBatch.metrics,
      alternateBatch.metrics,
    ),
  );
}

export function solveAimWithCollisionPitchHeuristic<
  TContext extends AimContext,
  TCandidate extends LaunchCandidate,
  TCollisionFreeInfo extends TrajectoryInfo,
  TCollisionAwareInfo extends TrajectoryInfo,
>(
  request: CollisionFallbackPitchSearchRequest<
    TContext,
    TCandidate,
    TCollisionFreeInfo,
    TCollisionAwareInfo
  >,
): AimSolution<TContext, TCandidate, TCollisionAwareInfo> | null {
  const initialBatch = solveBatchWithCollisions(
    request,
    request.initialCandidates,
  );

  if (initialBatch?.hit) {
    return initialBatch;
  }

  if (!request.alternateCandidates) {
    return initialBatch;
  }

  const alternateBatch = solveBatchWithCollisions(
    request,
    request.alternateCandidates,
  );

  return alternateBatch ?? initialBatch;
}
