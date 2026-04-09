import type {
  AimContext,
  LaunchCandidate,
  TrajectoryInfo,
  TrajectoryProvider,
  TrajectorySource,
  TrajectorySample,
} from "./types";

export interface FunctionalTrajectoryProviderOptions<
  TContext,
  TCandidate extends LaunchCandidate,
  TInfo extends TrajectoryInfo,
> {
  readonly getTrajectory: (
    context: TContext,
    candidate: TCandidate,
  ) => TrajectorySource<TInfo>;
}

export class FunctionalTrajectoryProvider<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
  TInfo extends TrajectoryInfo = TrajectoryInfo,
> implements TrajectoryProvider<TContext, TCandidate, TInfo>
{
  public constructor(
    private readonly options: FunctionalTrajectoryProviderOptions<TContext, TCandidate, TInfo>,
  ) {}

  public getTrajectory(
    context: TContext,
    candidate: TCandidate,
  ): TrajectorySource<TInfo> {
    return this.options.getTrajectory(context, candidate);
  }
}
