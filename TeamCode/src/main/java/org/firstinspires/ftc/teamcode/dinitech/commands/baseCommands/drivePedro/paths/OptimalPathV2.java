package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
    public static double ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES = 40.0;

    /** Below this range we skip tangent and use linear heading only. */
    public static double MIN_RANGE_FOR_PIECEWISE_INCHES = 50.0;

    public OptimalPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, pathSupplier, maxPower, holdEnd);
    }

    public static OptimalPath line(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
            (builder, startPose) -> {
                BezierLine line = new BezierLine(startPose, targetPose);
                double angleBetween = angleBetween(startPose, targetPose);
                return new PathPlan(
                    builder.addPath(line),
                    new PathMetrics(
                            angleBetween,
                            startPose.distanceFrom(targetPose),
                            t -> angleBetween));});

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath lineSpecificTangent(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, boolean useReverseTangent, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplierSpecificTangent(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierLine line = new BezierLine(startPose, targetPose);
                    return new PathPlan(
                            builder.addPath(line),
                            new PathMetrics(
                                    angleBetween(startPose, targetPose),
                                startPose.distanceFrom(targetPose),
                                t -> angleBetween(startPose, targetPose)));
                },
                useReverseTangent);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath triangle(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));});

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath triangleSpecificTangent(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, boolean useReverseTangent, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplierSpecificTangent(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));
                },
                useReverseTangent);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath cubic(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
            (builder, startPose) -> {
                BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, targetPose);
                return new PathPlan(
                    builder.addPath(curve),
                    new PathMetrics(
                        tangentAngleFromDerivative(curve, 0),
                        curve.approximateLength(),
                        t -> tangentAngleFromDerivative(curve, t)));});

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath cubicSpecificTangent(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, boolean useReverseTangent, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplierSpecificTangent(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));
                },
                useReverseTangent);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath penta(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, controlPoint3, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));});

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath pentaSpecificTangent(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose targetPose, boolean useReverseTangent, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplierSpecificTangent(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, controlPoint3, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));
                },
                useReverseTangent);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath octo(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose controlPoint4, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, controlPoint3, controlPoint4, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));});

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath octoSpecificTangent(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose controlPoint4, Pose targetPose, boolean useReverseTangent, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplierSpecificTangent(
                drivePedroSubsystem,
                targetPose,
                (builder, startPose) -> {
                    BezierCurve curve = new BezierCurve(startPose, controlPoint1, controlPoint2, controlPoint3, controlPoint4, targetPose);
                    return new PathPlan(
                            builder.addPath(curve),
                            new PathMetrics(
                                    tangentAngleFromDerivative(curve, 0),
                                curve.approximateLength(),
                                t -> tangentAngleFromDerivative(curve, t)));
                },
                useReverseTangent);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    private static PathSupplier createSupplier(
            DrivePedroSubsystem drivePedroSubsystem,
            Pose targetPose,
            PathPlanProvider pathPlanProvider
    ) {
        return builder -> {
            PathPlan pathPlan = pathPlanProvider.get(builder, drivePedroSubsystem.getPose());

            HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeading(
                    drivePedroSubsystem.getHeading(),
                    pathPlan.pathMetrics,
                    targetPose.getHeading(),
                    pathPlan.pathMetrics.range);

            return pathPlan.builder
                .setHeadingInterpolation(headingInterpolator)
                .setBrakingStrength(getBrakingStrengthScaleFromRange(pathPlan.pathMetrics.range))
                .build();
        };
    }

            private static PathSupplier createSupplierSpecificTangent(
                DrivePedroSubsystem drivePedroSubsystem,
                Pose targetPose,
                PathPlanProvider pathPlanProvider,
                boolean useReverseTangent
            ) {
            return builder -> {
                PathPlan pathPlan = pathPlanProvider.get(builder, drivePedroSubsystem.getPose());

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingSpecificTangent(
                    drivePedroSubsystem.getHeading(),
                    pathPlan.pathMetrics,
                    targetPose.getHeading(),
                    pathPlan.pathMetrics.range,
                    useReverseTangent);

                return pathPlan.builder
                    .setHeadingInterpolation(headingInterpolator)
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(pathPlan.pathMetrics.range))
                    .build();
            };
            }

    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeading(
            double startHeading,
            PathMetrics pathMetrics,
            double targetHeading,
            double range
    ) {
        HeadingPlan headingPlan = OptimizePieceWiseHeading.optimize(
                startHeading,
            pathMetrics,
                targetHeading,
                range);

        if (!headingPlan.usePiecewise) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(range));
        }

        double headingAtTangentStart = tangentHeadingAt(pathMetrics, headingPlan.tTangent, headingPlan.useReverseTangent);
        double headingAtFinalT = tangentHeadingAt(pathMetrics, headingPlan.tFinalHeading, headingPlan.useReverseTangent);

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0.0,
                        headingPlan.tTangent,
                HeadingInterpolator.linear(startHeading, headingAtTangentStart)),

                new HeadingInterpolator.PiecewiseNode(
                        headingPlan.tTangent,
                        headingPlan.tFinalHeading,
                        headingPlan.tangentInterpolator),

                new HeadingInterpolator.PiecewiseNode(
                        headingPlan.tFinalHeading,
                        1.0,
                    HeadingInterpolator.linear(headingAtFinalT, targetHeading))
        );
    }

            private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingSpecificTangent(
                double startHeading,
                PathMetrics pathMetrics,
                double targetHeading,
                double range,
                boolean useReverseTangent
            ) {
            HeadingPlan headingPlan = OptimizePieceWiseHeading.optimizeSpecificTangent(
                startHeading,
                pathMetrics,
                targetHeading,
                range,
                useReverseTangent);

            if (!headingPlan.usePiecewise) {
                return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(range));
            }

            double headingAtTangentStart = tangentHeadingAt(pathMetrics, headingPlan.tTangent, headingPlan.useReverseTangent);
            double headingAtFinalT = tangentHeadingAt(pathMetrics, headingPlan.tFinalHeading, headingPlan.useReverseTangent);

            return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                    0.0,
                    headingPlan.tTangent,
                    HeadingInterpolator.linear(startHeading, headingAtTangentStart)),

                new HeadingInterpolator.PiecewiseNode(
                    headingPlan.tTangent,
                    headingPlan.tFinalHeading,
                    headingPlan.tangentInterpolator),

                new HeadingInterpolator.PiecewiseNode(
                    headingPlan.tFinalHeading,
                    1.0,
                    HeadingInterpolator.linear(headingAtFinalT, targetHeading))
            );
            }

    private static final class OptimizePieceWiseHeading {

        public static int T_SEARCH_STEPS = 240;

        static HeadingPlan optimize(double startHeading, PathMetrics pathMetrics, double targetHeading, double range) {
            if (range < MIN_RANGE_FOR_PIECEWISE_INCHES) {
                return HeadingPlan.linearOnly();
            }

            CandidatePlan forward = buildCandidate(startHeading, targetHeading, false, pathMetrics, range);
            CandidatePlan reverse = buildCandidate(startHeading, targetHeading, true, pathMetrics, range);

            CandidatePlan best = selectBest(forward, reverse);
            if (!best.validForPiecewise) {
                return HeadingPlan.linearOnly();
            }

            return new HeadingPlan(
                    true,
                    best.tangentInterpolator,
                    best.tTangent,
                    best.tFinalHeading,
                    best.useReverseTangent);

        }

        static HeadingPlan optimizeSpecificTangent(double startHeading, PathMetrics pathMetrics, double targetHeading, double range, boolean useReverseTangent) {
            if (range < MIN_RANGE_FOR_PIECEWISE_INCHES) {
                return HeadingPlan.linearOnly();
            }

            CandidatePlan forward = buildCandidate(startHeading, targetHeading, false, pathMetrics, range);
            CandidatePlan reverse = buildCandidate(startHeading, targetHeading, true, pathMetrics, range);

            CandidatePlan forced = useReverseTangent ? reverse : forward;
            if (!forced.validForPiecewise) {
                return HeadingPlan.linearOnly();
            }

            return new HeadingPlan(
                    true,
                    forced.tangentInterpolator,
                    forced.tTangent,
                    forced.tFinalHeading,
                    forced.useReverseTangent);
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
                double targetHeading,
                boolean reverse,
                PathMetrics pathMetrics,
                double range
        ) {
            double tTangent = findTangentEntryTime(startHeading, reverse, pathMetrics, range);
            double tFinalHeading = findFinalAlignmentStartTime(targetHeading, reverse, pathMetrics, range);

            double tangentSpan = Math.max(0.0, tFinalHeading - tTangent);
            boolean validForPiecewise = tangentSpan > 1e-4;

            double headingAtTangentStart = tangentHeadingAt(pathMetrics, tTangent, reverse);
            double headingAtFinalT = tangentHeadingAt(pathMetrics, tFinalHeading, reverse);

            double deltaStart = smallestAngleDifference(headingAtTangentStart, startHeading);
            double deltaEnd = smallestAngleDifference(targetHeading, headingAtFinalT);

            return new CandidatePlan(
                    reverse ? HeadingInterpolator.tangent.reverse() : HeadingInterpolator.tangent,
                    validForPiecewise,
                    tTangent,
                    tFinalHeading,
                    tangentSpan,
                    Math.abs(deltaStart) + Math.abs(deltaEnd),
                    reverse);
        }

        private static double findTangentEntryTime(
                double startHeading,
                boolean reverse,
                PathMetrics pathMetrics,
                double range
        ) {
            for (int i = 0; i <= T_SEARCH_STEPS; i++) {
                double t = i / (double) T_SEARCH_STEPS;
                double tangentHeading = tangentHeadingAt(pathMetrics, t, reverse);
                double deltaStart = smallestAngleDifference(tangentHeading, startHeading);
                if (requiredFractionForRotation(deltaStart, range) <= t) {
                    return t;
                }
            }
            return 1.0;
        }

        private static double findFinalAlignmentStartTime(
                double targetHeading,
                boolean reverse,
                PathMetrics pathMetrics,
                double range
        ) {
            for (int i = T_SEARCH_STEPS; i >= 0; i--) {
                double t = i / (double) T_SEARCH_STEPS;
                double tangentHeading = tangentHeadingAt(pathMetrics, t, reverse);
                double deltaEnd = smallestAngleDifference(targetHeading, tangentHeading);
                if (requiredFractionForRotation(deltaEnd, range) <= (1.0 - t)) {
                    return t;
                }
            }
            return 0.0;
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

    private static double tangentAngleFromDerivative(BezierCurve curve, double t) {
        Vector firstDerivative = curve.getDerivative(t);
        return Math.atan2(firstDerivative.getYComponent(), firstDerivative.getXComponent());
    }

    private static double tangentHeadingAt(PathMetrics pathMetrics, double t, boolean useReverseTangent) {
        double tangentHeading = pathMetrics.tangentAngleAtTProvider.getTangentAngleAt(t);
        return normalizeAngle(tangentHeading + (useReverseTangent ? Math.PI : 0.0));
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    @FunctionalInterface
    private interface PathPlanProvider {
        PathPlan get(PathBuilder builder, Pose startPose);
    }

    private static final class PathPlan {
        final PathBuilder builder;
        final PathMetrics pathMetrics;

        PathPlan(PathBuilder builder, PathMetrics pathMetrics) {
            this.builder = builder;
            this.pathMetrics = pathMetrics;
        }
    }

    private static final class PathMetrics {
        final double tangentAngle;
        final double range;
        final TangentAngleAtTProvider tangentAngleAtTProvider;

        PathMetrics(double tangentAngle, double range, TangentAngleAtTProvider tangentAngleAtTProvider) {
            this.tangentAngle = tangentAngle;
            this.range = range;
            this.tangentAngleAtTProvider = tangentAngleAtTProvider;
        }
    }

    private static final class HeadingPlan {
        final boolean usePiecewise;
        final HeadingInterpolator tangentInterpolator;
        final double tTangent;
        final double tFinalHeading;
        final boolean useReverseTangent;

        HeadingPlan(
                boolean usePiecewise,
                HeadingInterpolator tangentInterpolator,
                double tTangent,
                double tFinalHeading,
                boolean useReverseTangent
        ) {
            this.usePiecewise = usePiecewise;
            this.tangentInterpolator = tangentInterpolator;
            this.tTangent = tTangent;
            this.tFinalHeading = tFinalHeading;
            this.useReverseTangent = useReverseTangent;
        }

        static HeadingPlan linearOnly() {
            return new HeadingPlan(false, HeadingInterpolator.tangent, 0.0, 1.0, false);
        }
    }

    private static final class CandidatePlan {
        final HeadingInterpolator tangentInterpolator;
        final boolean validForPiecewise;
        final double tTangent;
        final double tFinalHeading;
        final double tangentSpan;
        final double totalRotation;
        final boolean useReverseTangent;

        CandidatePlan(
                HeadingInterpolator tangentInterpolator,
                boolean validForPiecewise,
                double tTangent,
                double tFinalHeading,
                double tangentSpan,
                double totalRotation,
                boolean useReverseTangent
        ) {
            this.tangentInterpolator = tangentInterpolator;
            this.validForPiecewise = validForPiecewise;
            this.tTangent = tTangent;
            this.tFinalHeading = tFinalHeading;
            this.tangentSpan = tangentSpan;
            this.totalRotation = totalRotation;
            this.useReverseTangent = useReverseTangent;
        }
    }

    @FunctionalInterface
    private interface TangentAngleAtTProvider {
        double getTangentAngleAt(double t);
    }
}