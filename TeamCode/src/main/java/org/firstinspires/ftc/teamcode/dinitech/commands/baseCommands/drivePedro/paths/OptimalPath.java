package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static com.pedropathing.math.MathFunctions.getSmallestAngleDifference;
import static com.pedropathing.math.MathFunctions.normalizeAngle;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getLinearInterpolationHeadingEndTimeFromRange;

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
public class OptimalPath extends FollowPath {
    /** Distance needed for a full PI radians rotation. */
    public static double ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES = 45.0;
    public static double DELTA_T = 0.01;
    public static int nDCompute = 3;

    private static double getTangentAtTCurve(BezierCurve curve, double t) {
        return curve.getDerivative(t).getTheta();
    }

    private static double tangentAtLine(BezierLine line) {
        return line.getEndTangent().getTheta();
    }
    
    private static double getDistanceNeededToRotate(double angle) {
        return ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES * (Math.abs(angle) / Math.PI);
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static double getTangentHeadingAt(BezierCurve curve, double t, boolean forwardTangent) {
        double tangent = getTangentAtTCurve(curve, clamp01(t));
        return forwardTangent ? tangent : normalizeAngle(tangent + Math.PI);
    }


    public OptimalPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, pathSupplier, maxPower, holdEnd);
    }

    public static OptimalPath line(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createLineSupplier(
                drivePedroSubsystem,
                (currentPose, currentHeading) -> targetPose);

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    protected static PathSupplier createLineSupplier(
            DrivePedroSubsystem drivePedroSubsystem,
            TargetPoseProvider targetPoseProvider
    ) {
        return createSupplier(
                drivePedroSubsystem,
                (builder, currentPose, currentHeading) -> {
                    Pose targetPose = targetPoseProvider.getTargetPose(currentPose, currentHeading);
                    BezierLine line = new BezierLine(currentPose, targetPose);
                    double range = currentPose.distanceFrom(targetPose);

                    HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingLine(
                            currentHeading,
                            line,
                            targetPose.getHeading(),
                            range);

                    return new PathPlan(builder.addPath(line), headingInterpolator, range);
                });
    }

    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingLine(
            double startHeading,
            BezierLine line,
            double targetHeading,
            double fullPathRange
    ){

        if (fullPathRange <= 1e-6) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        double forwardTangentAtLine = tangentAtLine(line);
        double reverseTangentAtLine = normalizeAngle(forwardTangentAtLine + Math.PI);

        double startDeltaForward = getSmallestAngleDifference(forwardTangentAtLine, startHeading);
        double endDeltaForward = getSmallestAngleDifference(forwardTangentAtLine, targetHeading);

        double startDeltaBackward = getSmallestAngleDifference(reverseTangentAtLine, startHeading);
        double endDeltaBackward= getSmallestAngleDifference(reverseTangentAtLine, targetHeading);

        // Minimize total rotation by picking tangent or reverse tangent based on which has smaller total angle change.
        boolean forwardTangent = !(startDeltaForward + endDeltaForward > startDeltaBackward + endDeltaBackward);

        double distanceForStartDelta;
        double distanceForEndDelta;
        double tangent;

        if (forwardTangent) {
            distanceForStartDelta = getDistanceNeededToRotate(startDeltaForward);
            distanceForEndDelta = getDistanceNeededToRotate(endDeltaForward);
            tangent = forwardTangentAtLine;

        } else {
            distanceForStartDelta = getDistanceNeededToRotate(startDeltaBackward);
            distanceForEndDelta = getDistanceNeededToRotate(endDeltaBackward);
            tangent = reverseTangentAtLine;

        }

        if (distanceForStartDelta + distanceForEndDelta > fullPathRange) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        // Now compute tTangent and tFinalHeading
        double tTangentBegin = clamp01(distanceForStartDelta / fullPathRange);
        double tTangentEnd = clamp01(1 - distanceForEndDelta / fullPathRange);

        if (tTangentEnd - tTangentBegin <= 1e-4) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, tTangentBegin,
                        HeadingInterpolator.linear(startHeading, tangent,
                                getLinearInterpolationHeadingEndTimeFromRange(distanceForStartDelta))),
                new HeadingInterpolator.PiecewiseNode(tTangentBegin, tTangentEnd,
                        forwardTangent ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(tTangentEnd, 1,
                        HeadingInterpolator.linear(tangent, targetHeading,
                                getLinearInterpolationHeadingEndTimeFromRange(distanceForEndDelta))));
    }

    public static OptimalPath curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
            (builder, currentPose, currentHeading) -> {
                BezierCurve curve = new BezierCurve(currentPose, controlPoint1, targetPose);
                double range = curve.approximateLength();

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    currentHeading,
                    curve,
                    targetPose.getHeading(),
                    range);

                return new PathPlan(builder.addPath(curve), headingInterpolator, range);
            });

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }
    public static OptimalPath curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
            (builder, currentPose, currentHeading) -> {
                BezierCurve curve = new BezierCurve(currentPose, controlPoint1, controlPoint2, targetPose);
                double range = curve.approximateLength();

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    currentHeading,
                    curve,
                    targetPose.getHeading(),
                    range);

                return new PathPlan(builder.addPath(curve), headingInterpolator, range);
            });

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPath curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
            (builder, currentPose, currentHeading) -> {
                BezierCurve curve = new BezierCurve(currentPose, controlPoint1, controlPoint2, controlPoint3, targetPose);
                double range = curve.length();

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    currentHeading,
                    curve,
                    targetPose.getHeading(),
                    range);

                return new PathPlan(builder.addPath(curve), headingInterpolator, range);
            });

        return new OptimalPath(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

        private static PathSupplier createSupplier(
            DrivePedroSubsystem drivePedroSubsystem,
            PathPlanProvider pathPlanProvider
        ) {
        return builder -> {
            PathPlan pathPlan = pathPlanProvider.get(builder, drivePedroSubsystem.getPose(), drivePedroSubsystem.getHeading());
            return pathPlan.builder
                .setHeadingInterpolation(pathPlan.headingInterpolator)
                .setBrakingStrength(getBrakingStrengthScaleFromRange(pathPlan.range))
                .build();
        };
        }


    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingCurve(
            double startHeading,
            BezierCurve curve,
            double targetHeading,
            double fullPathRange
    ) {

        if (fullPathRange <= 1e-6) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        double tBeginSample = clamp01(DELTA_T);
        double tEndSample = clamp01(1 - DELTA_T);

        double forwardTangentAtBeginCurve = getTangentAtTCurve(curve, tBeginSample);
        double reverseTangentAtBeginCurve = normalizeAngle(forwardTangentAtBeginCurve + Math.PI);

        double forwardTangentAtEndCurve = getTangentAtTCurve(curve, tEndSample);
        double reverseTangentAtEndCurve = normalizeAngle(forwardTangentAtEndCurve + Math.PI);

        double startDeltaForward = getSmallestAngleDifference(forwardTangentAtBeginCurve, startHeading);
        double endDeltaForward = getSmallestAngleDifference(forwardTangentAtEndCurve, targetHeading);

        double startDeltaBackward = getSmallestAngleDifference(reverseTangentAtBeginCurve, startHeading);
        double endDeltaBackward = getSmallestAngleDifference(reverseTangentAtEndCurve, targetHeading);

        boolean forwardTangent = !(startDeltaForward + endDeltaForward > startDeltaBackward + endDeltaBackward);

        double distanceForStartDelta;
        double distanceForEndDelta;

        if (forwardTangent) {
            distanceForStartDelta = getDistanceNeededToRotate(startDeltaForward);
            distanceForEndDelta = getDistanceNeededToRotate(endDeltaForward);

        } else {
            distanceForStartDelta = getDistanceNeededToRotate(startDeltaBackward);
            distanceForEndDelta = getDistanceNeededToRotate(endDeltaBackward);

        }

        if (distanceForStartDelta + distanceForEndDelta > fullPathRange) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        double tTangentBegin = clamp01(distanceForStartDelta / fullPathRange);
        double tTangentEnd = clamp01(1 - distanceForEndDelta / fullPathRange);

        if (tTangentEnd - tTangentBegin <= 1e-4) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        double tangentBegin = getTangentHeadingAt(curve, tTangentBegin, forwardTangent);
        double tangentEnd = getTangentHeadingAt(curve, tTangentEnd, forwardTangent);

        for (int i = 0 ; i < nDCompute ; i++){
            double startDelta = getSmallestAngleDifference(tangentBegin, startHeading);
            double endDelta = getSmallestAngleDifference(tangentEnd, targetHeading);

            distanceForStartDelta = getDistanceNeededToRotate(startDelta);
            distanceForEndDelta = getDistanceNeededToRotate(endDelta);

            tTangentBegin = clamp01(distanceForStartDelta / fullPathRange);
            tTangentEnd = clamp01(1 - distanceForEndDelta / fullPathRange);

            if (tTangentEnd - tTangentBegin <= 1e-4) {
                return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
            }

            tangentBegin = getTangentHeadingAt(curve, tTangentBegin, forwardTangent);
            tangentEnd = getTangentHeadingAt(curve, tTangentEnd, forwardTangent);
        }

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, tTangentBegin,
                        HeadingInterpolator.linear(startHeading, tangentBegin,
                                getLinearInterpolationHeadingEndTimeFromRange(distanceForStartDelta))),

            new HeadingInterpolator.PiecewiseNode(tTangentBegin, tTangentEnd,
                    forwardTangent ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()),

                new HeadingInterpolator.PiecewiseNode(tTangentEnd, 1,
                        HeadingInterpolator.linear(tangentEnd, targetHeading,
                                getLinearInterpolationHeadingEndTimeFromRange(distanceForEndDelta)))
        );

    }


    @FunctionalInterface
    private interface PathPlanProvider {
        PathPlan get(PathBuilder builder, Pose startPose, double startHeading);
    }

    @FunctionalInterface
    protected interface TargetPoseProvider {
        Pose getTargetPose(Pose currentPose, double currentHeading);
    }

    private static final class PathPlan {
        final PathBuilder builder;
        final HeadingInterpolator headingInterpolator;
        final double range;

        PathPlan(PathBuilder builder, HeadingInterpolator headingInterpolator, double range) {
            this.builder = builder;
            this.headingInterpolator = headingInterpolator;
            this.range = range;
        }
    }

}