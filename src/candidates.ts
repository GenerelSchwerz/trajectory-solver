import type {
  AimContext,
  CandidateGenerator,
  LaunchCandidate,
  YawProvider,
} from "./types";
import { subtractVectors, yawPitchFromDirection } from "./vector";

export interface AngleRange {
  readonly start: number;
  readonly end: number;
  readonly step: number;
}

export interface GridCandidateGeneratorOptions<TContext = AimContext> {
  readonly pitch: AngleRange;
  readonly yaw?: AngleRange;
  readonly yawSearch?: AngleRange;
  readonly fixedYaw?: number;
  readonly speed?: number;
  readonly yawMode?: "absolute" | "relative";
  readonly provideYaw?: YawProvider<TContext>;
}

function getDefaultYaw(context: AimContext): number {
  return yawPitchFromDirection(
    subtractVectors(
      context.target.referencePoint ?? context.origin,
      context.origin,
    ),
  ).yawRadians;
}

function getYawBase<TContext>(
  options: GridCandidateGeneratorOptions<TContext>,
  context: TContext,
): number {
  return options.yawMode === "relative"
    ? (options.provideYaw?.(context) ?? getDefaultYaw(context as AimContext))
    : 0;
}

function getYawValues<TContext>(
  options: GridCandidateGeneratorOptions<TContext>,
  context: TContext,
): number[] {
  const yawBase = getYawBase(options, context);

  if (options.fixedYaw !== undefined) {
    return [
      options.yawMode === "relative"
        ? yawBase + options.fixedYaw
        : options.fixedYaw,
    ];
  }

  const yawRange = options.yawSearch ?? options.yaw;

  if (yawRange) {
    const yawValues: number[] = [];

    for (
      let yawValue = yawRange.start;
      yawValue <= yawRange.end;
      yawValue += yawRange.step
    ) {
      yawValues.push(
        options.yawMode === "relative"
          ? yawBase + yawValue
          : yawValue,
      );
    }

    return yawValues;
  }

  if (options.provideYaw || options.yawMode === "relative") {
    return [yawBase];
  }

  return [];
}

export class GridCandidateGenerator<
  TContext = AimContext,
  TCandidate extends LaunchCandidate = LaunchCandidate,
> implements CandidateGenerator<TContext, TCandidate>
{
  public constructor(
    private readonly options: GridCandidateGeneratorOptions<TContext>,
    private readonly mapCandidate?: (candidate: LaunchCandidate) => TCandidate,
  ) {}

  public *getCandidates(context: TContext): Iterable<TCandidate> {
    const yawValues = getYawValues(this.options, context);

    for (const yawRadians of yawValues) {
      for (
        let pitchRadians = this.options.pitch.start;
        pitchRadians <= this.options.pitch.end;
        pitchRadians += this.options.pitch.step
      ) {
        const candidate: LaunchCandidate = {
          yawRadians,
          pitchRadians,
          speed: this.options.speed,
        };

        yield this.mapCandidate ? this.mapCandidate(candidate) : (candidate as TCandidate);
      }
    }
  }
}
