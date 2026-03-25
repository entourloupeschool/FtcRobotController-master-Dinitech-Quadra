package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static com.pedropathing.math.MathFunctions.getSmallestAngleDifference;
import static com.pedropathing.math.MathFunctions.normalizeAngle;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

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
public final class OptimalPathV2 extends FollowPath {
    /** Distance needed for a full PI radians rotation. */
    public static double ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES = 30.0;

    private static double getTangentAtTCurve(BezierCurve curve, double t) {
        return curve.getDerivative(t).getTheta();
    }

    private static double tangentAtLine(BezierLine line) {
        return line.getEndTangent().getTheta();
    }
    
    private static double getDistanceNeededToRotate(double angle) {
        return ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES * (angle / Math.PI);
    }

    private static double getTTangentFromRange(double range, double fullPathRange) {
        return getDistanceNeededToRotate(range) / fullPathRange;
    }


    public OptimalPathV2(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, pathSupplier, maxPower, holdEnd);
    }

    public static OptimalPathV2 line(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
            (builder, currentPose, currentHeading) -> {
                BezierLine line = new BezierLine(currentPose, targetPose);
                double range = currentPose.distanceFrom(targetPose);

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingLine(
                    currentHeading,
                    line,
                    targetPose.getHeading(),
                    range);

                return new PathPlan(builder.addPath(line), headingInterpolator, range);
            });

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingLine(
            double startHeading,
            BezierLine line,
            double targetHeading,
            double fullPathRange
    ){

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
        double tTangent = getTTangentFromRange(distanceForStartDelta, fullPathRange);
        double tFinalHeading = 1 - getTTangentFromRange(distanceForEndDelta, fullPathRange);
        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, tTangent,
                        HeadingInterpolator.linear(startHeading, tangent)),
                new HeadingInterpolator.PiecewiseNode(tTangent, tFinalHeading,
                        forwardTangent ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(tFinalHeading, 1,
                        HeadingInterpolator.linear(tangent, targetHeading)));
    }

    public static OptimalPathV2 curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
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

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }
    public static OptimalPathV2 curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, double maxPower, boolean holdEnd) {
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

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    public static OptimalPathV2 curve(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose controlPoint3, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createSupplier(
                drivePedroSubsystem,
            (builder, currentPose, currentHeading) -> {
                BezierCurve curve = new BezierCurve(currentPose, controlPoint1, controlPoint2, controlPoint3, targetPose);
                double range = curve.approximateLength();

                HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    currentHeading,
                    curve,
                    targetPose.getHeading(),
                    range);

                return new PathPlan(builder.addPath(curve), headingInterpolator, range);
            });

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
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

        double forwardTangentAtBeginCurve = getTangentAtTCurve(curve, 0);
        double reverseTangentAtBeginCurve = normalizeAngle(forwardTangentAtBeginCurve + Math.PI);

        double forwardTangentAtEndCurve = getTangentAtTCurve(curve, 1);
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

        double tTangentBegin = getTTangentFromRange(distanceForStartDelta, fullPathRange);
        double tangentBegin = getTangentAtTCurve(curve, tTangentBegin);

        double tTangentEnd = 1 - getTTangentFromRange(distanceForEndDelta, fullPathRange);
        double tangentEnd = getTangentAtTCurve(curve, tTangentEnd);

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, tTangentBegin,
                        HeadingInterpolator.linear(startHeading, tangentBegin)),

                new HeadingInterpolator.PiecewiseNode(tangentBegin, tTangentEnd, forwardTangent ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()),

                new HeadingInterpolator.PiecewiseNode(tTangentEnd, 1,
                        HeadingInterpolator.linear(tangentEnd, targetHeading)));

    }


    @FunctionalInterface
    private interface PathPlanProvider {
        PathPlan get(PathBuilder builder, Pose startPose, double startHeading);
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