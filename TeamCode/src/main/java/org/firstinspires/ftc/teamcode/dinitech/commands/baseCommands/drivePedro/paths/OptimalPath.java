package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

/**
 * Path builder tuned to maximize tangent heading travel while minimizing rotation demand.
 *
 * <p>Rules:
 * <ol>
 *   <li>If range is short, use pure linear heading interpolation.</li>
 *   <li>If range is long enough, split heading into:
 *     rotate to tangent -> tangent/reverse tangent -> rotate to final heading.</li>
 *   <li>Pick tangent or reverse-tangent based on the smallest total heading change.</li>
 * </ol>
 */
@Configurable
public final class OptimalPath extends FollowPath {

    /** Distance needed for a full PI radians rotation. */
    public static double ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES = 55.0;

    /** Below this range we skip tangent and use linear heading only. */
    public static double MIN_RANGE_FOR_PIECEWISE_INCHES = 50.0;

    public OptimalPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, pathSupplier, maxPower, holdEnd);
    }

    public static OptimalPath line(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> builder.addPath(new BezierLine(startPose, targetPose)),
                startPose -> angleBetween(startPose, targetPose)
        );
        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath quadratic(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> builder.addPath(new BezierCurve(startPose, controlPoint1, targetPose)),
                // Bezier control-point heading is intentionally ignored; only geometry matters.
                startPose -> angleBetween(startPose, controlPoint1)
        );
        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath cubic(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> builder.addPath(new BezierCurve(startPose, controlPoint1, controlPoint2, targetPose)),
                // Tangent at start of cubic is defined by start -> controlPoint1.
                startPose -> angleBetween(startPose, controlPoint1)
        );
        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    private static PathSupplier createSupplier(
            DrivePedroSubsystem drivePedroSubsystem,
            Pose targetPose,
            PathBuilderConfigurator pathConfigurator,
            TangentAngleProvider tangentAngleProvider
    ) {
        return builder -> {
            Pose startPose = drivePedroSubsystem.getPose();
            double startHeading = drivePedroSubsystem.getHeading();
            double range = startPose.distanceFrom(targetPose);
            double tangentAngle = tangentAngleProvider.get(startPose);

            HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeading(
                    startHeading,
                    tangentAngle,
                    targetPose.getHeading(),
                    range
            );

            return pathConfigurator.configure(builder, startPose)
                    .setHeadingInterpolation(headingInterpolator)
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();
        };
    }

    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeading(
            double startHeading,
            double tangentAngle,
            double targetHeading,
            double range
    ) {
        HeadingPlan headingPlan = OptimizePieceWiseHeading.optimize(
                startHeading,
                tangentAngle,
                targetHeading,
                range
        );

        if (!headingPlan.usePiecewise) {
            return HeadingInterpolator.linear(startHeading, targetHeading);
        }

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0.0,
                        headingPlan.tTangent,
                        HeadingInterpolator.linear(startHeading, headingPlan.effectiveTangentHeading)
                ),
                new HeadingInterpolator.PiecewiseNode(
                        headingPlan.tTangent,
                        headingPlan.tFinalHeading,
                        headingPlan.tangentInterpolator
                ),
                new HeadingInterpolator.PiecewiseNode(
                        headingPlan.tFinalHeading,
                        1.0,
                        HeadingInterpolator.linear(headingPlan.effectiveTangentHeading, targetHeading)
                )
        );
    }

    private static final class OptimizePieceWiseHeading {

        static HeadingPlan optimize(double startHeading, double tangentAngle, double targetHeading, double range) {
            if (range < MIN_RANGE_FOR_PIECEWISE_INCHES) {
                return HeadingPlan.linearOnly();
            }

            CandidatePlan forward = buildCandidate(startHeading, normalizeAngle(tangentAngle), targetHeading, false, range);
            CandidatePlan reverse = buildCandidate(startHeading, normalizeAngle(tangentAngle + Math.PI), targetHeading, true, range);

            CandidatePlan best = selectBest(forward, reverse);
            if (!best.validForPiecewise) {
                return HeadingPlan.linearOnly();
            }

            return new HeadingPlan(
                    true,
                    best.effectiveTangentHeading,
                    best.tangentInterpolator,
                    best.tTangent,
                    best.tFinalHeading
            );
        }

        private static CandidatePlan selectBest(CandidatePlan a, CandidatePlan b) {
            if (a.validForPiecewise != b.validForPiecewise) {
                return a.validForPiecewise ? a : b;
            }
            if (a.totalRotation < b.totalRotation) {
                return a;
            }
            if (b.totalRotation < a.totalRotation) {
                return b;
            }
            return a.tangentSpan >= b.tangentSpan ? a : b;
        }

        private static CandidatePlan buildCandidate(
                double startHeading,
                double effectiveTangentHeading,
                double targetHeading,
                boolean reverse,
                double range
        ) {
            double deltaStart = smallestAngleDifference(effectiveTangentHeading, startHeading);
            double deltaEnd = smallestAngleDifference(targetHeading, effectiveTangentHeading);

            double tRotateToTangent = requiredFractionForRotation(deltaStart, range);
            double tRotateToFinal = requiredFractionForRotation(deltaEnd, range);
            double totalRotationFraction = tRotateToTangent + tRotateToFinal;

            double tTangent = clamp01(tRotateToTangent);
            double tFinalHeading = clamp01(1.0 - tRotateToFinal);
            if (tFinalHeading < tTangent) {
                tFinalHeading = tTangent;
            }

            double tangentSpan = Math.max(0.0, tFinalHeading - tTangent);
            boolean validForPiecewise = totalRotationFraction < 1.0 && tangentSpan > 1e-4;

            return new CandidatePlan(
                    effectiveTangentHeading,
                    reverse ? HeadingInterpolator.tangent.reverse() : HeadingInterpolator.tangent,
                    validForPiecewise,
                    tTangent,
                    tFinalHeading,
                    tangentSpan,
                    Math.abs(deltaStart) + Math.abs(deltaEnd)
            );
        }
    }

    private static double requiredFractionForRotation(double angleDelta, double range) {
        if (range <= 1e-6) {
            return 1.0;
        }
        double requiredDistance = ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES * (Math.abs(angleDelta) / Math.PI);
        return requiredDistance / range;
    }

    private static double smallestAngleDifference(double targetAngle, double currentAngle) {
        return Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double angleBetween(Pose from, Pose to) {
        return Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    @FunctionalInterface
    private interface PathBuilderConfigurator {
        PathBuilder configure(PathBuilder builder, Pose startPose);
    }

    @FunctionalInterface
    private interface TangentAngleProvider {
        double get(Pose startPose);
    }

    private static final class HeadingPlan {
        final boolean usePiecewise;
        final double effectiveTangentHeading;
        final HeadingInterpolator tangentInterpolator;
        final double tTangent;
        final double tFinalHeading;

        HeadingPlan(
                boolean usePiecewise,
                double effectiveTangentHeading,
                HeadingInterpolator tangentInterpolator,
                double tTangent,
                double tFinalHeading
        ) {
            this.usePiecewise = usePiecewise;
            this.effectiveTangentHeading = effectiveTangentHeading;
            this.tangentInterpolator = tangentInterpolator;
            this.tTangent = tTangent;
            this.tFinalHeading = tFinalHeading;
        }

        static HeadingPlan linearOnly() {
            return new HeadingPlan(false, 0.0, HeadingInterpolator.tangent, 0.0, 1.0);
        }
    }

    private static final class CandidatePlan {
        final double effectiveTangentHeading;
        final HeadingInterpolator tangentInterpolator;
        final boolean validForPiecewise;
        final double tTangent;
        final double tFinalHeading;
        final double tangentSpan;
        final double totalRotation;

        CandidatePlan(
                double effectiveTangentHeading,
                HeadingInterpolator tangentInterpolator,
                boolean validForPiecewise,
                double tTangent,
                double tFinalHeading,
                double tangentSpan,
                double totalRotation
        ) {
            this.effectiveTangentHeading = effectiveTangentHeading;
            this.tangentInterpolator = tangentInterpolator;
            this.validForPiecewise = validForPiecewise;
            this.tTangent = tTangent;
            this.tFinalHeading = tFinalHeading;
            this.tangentSpan = tangentSpan;
            this.totalRotation = totalRotation;
        }
    }
}