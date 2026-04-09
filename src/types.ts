import type { Vector3 } from "./vector";

export interface LaunchAngles {
  readonly yawRadians: number;
  readonly pitchRadians: number;
}

export interface LaunchCandidate extends LaunchAngles {
  readonly speed?: number;
  readonly label?: string;
  readonly metadata?: Record<string, unknown>;
}

export interface TrajectorySample {
  readonly tick: number;
  readonly position: Vector3;
  readonly velocity?: Vector3;
}

export interface SegmentHit {
  readonly hit: boolean;
  readonly point?: Vector3;
  readonly face?: string;
  readonly normal?: Vector3;
  readonly metadata?: Record<string, unknown>;
}

export interface TargetGeometry {
  contains?(point: Vector3): boolean;
  intersects?(point: Vector3): boolean;
  isHit?(point: Vector3): boolean;
  intersectsSegment?(start: Vector3, end: Vector3): boolean;
  traceSegment?(start: Vector3, end: Vector3): SegmentHit | null;
  distanceTo?(point: Vector3): number;
  idealPoint?: Vector3;
  referencePoint?: Vector3;
  metadata?: Record<string, unknown>;
}

export interface PiecewiseTrajectoryInfo {
  readonly kind: "piecewise";
  readonly sampleCount?: number;
  readonly timeStep?: number;
  readonly integrationMethod?: string;
  readonly terminatedEarly?: boolean;
  readonly metadata?: Record<string, unknown>;
}

export interface FormulaicTrajectoryInfo {
  readonly kind: "formulaic";
  readonly equation?: string;
  readonly closedForm?: boolean;
  readonly solvedFor?: string;
  readonly metadata?: Record<string, unknown>;
}

export interface ConditionalTrajectoryInfo {
  readonly kind: "condition";
  readonly condition: string;
  readonly solver?: string;
  readonly iterations?: number;
  readonly converged?: boolean;
  readonly rootValue?: number;
  readonly metadata?: Record<string, unknown>;
}

export type TrajectoryInfo =
  | PiecewiseTrajectoryInfo
  | FormulaicTrajectoryInfo
  | ConditionalTrajectoryInfo;

export interface TrajectoryComputation<
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> {
  readonly samples: Iterable<TrajectorySample>;
  readonly info: TInfo;
}

export type TrajectorySource<
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> = Iterable<TrajectorySample> | TrajectoryComputation<TInfo>;

export interface TrajectoryTarget {
  readonly geometry: TargetGeometry;
  readonly idealPoint?: Vector3;
  readonly referencePoint?: Vector3;
  readonly metadata?: Record<string, unknown>;
}

export interface AimContext {
  readonly origin: Vector3;
  readonly target: TrajectoryTarget;
  readonly originVelocity?: Vector3;
  readonly metadata?: Record<string, unknown>;
}

export type YawProvider<TContext = AimContext> = (context: TContext) => number;

export interface TrajectoryProvider<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> {
  getTrajectory(
    context: TContext,
    candidate: TCandidate,
  ): TrajectorySource<TInfo>;
}

export interface CandidateGenerator<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
> {
  getCandidates(context: TContext): Iterable<TCandidate>;
}

export interface AimEvaluation {
  readonly hit: boolean;
  readonly distance: number;
  readonly closestSample: TrajectorySample;
  readonly hitPoint?: Vector3;
  readonly hitFace?: string;
}

export interface EvaluatedTrajectory<
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> {
  readonly samples: Iterable<TrajectorySample>;
  readonly info: TInfo;
}

export interface AimSolution<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> {
  readonly hit: boolean;
  readonly yawRadians: number;
  readonly pitchRadians: number;
  readonly direction: Vector3;
  readonly distance: number;
  readonly closestPosition: Vector3;
  readonly tick: number;
  readonly candidate: TCandidate;
  readonly context: TContext;
  readonly trajectory: EvaluatedTrajectory<TInfo>;
}

export interface AimSearchRequest<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> {
  readonly context: TContext;
  readonly provider: TrajectoryProvider<TContext, TCandidate, TInfo>;
  readonly candidates: Iterable<TCandidate>;
}

export interface PitchHeuristicIteration {
  readonly iteration: number;
  readonly pitchRadians: number;
  readonly deltaRadians: number;
  readonly distance: number;
  readonly hit: boolean;
}

export interface PitchHeuristicSearchRequest<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> extends AimSearchRequest<TContext, TCandidate, TInfo> {
  readonly initialDeltaRadians: number;
  readonly maxIterations?: number;
  readonly createCandidate?: (input: {
    baseCandidate: TCandidate;
    pitchRadians: number;
    deltaRadians: number;
    iteration: number;
    history: readonly PitchHeuristicIteration[];
  }) => TCandidate;
}
