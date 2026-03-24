package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static com.pedropathing.math.MathFunctions.getSmallestAngleDifference;
import static com.pedropathing.math.MathFunctions.normalizeAngle;
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
public final class OptimalPathV2 extends FollowPath {
    private static double tangentHeadingAtTCurve(BezierCurve curve, double t) {
        return curve.getDerivative(t).getTheta();
    }
    private static double getDistanceNeededToRotate(double angle) {
        return ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES * (angle / Math.PI);
    }

    private static double getTTangentFromRange(double range, double fullPathRange) {
        return getDistanceNeededToRotate(range) / fullPathRange;
    }

    /** Distance needed for a full PI radians rotation. */
    public static double ROTATION_DISTANCE_FOR_PI_RADIANS_INCHES = 40.0;

    /** Below this range we skip tangent and use linear heading only. */
    public static double MIN_RANGE_FOR_PIECEWISE_INCHES = 50.0;

    public static int T_SEARCH_STEPS = 240;

    public OptimalPathV2(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, pathSupplier, maxPower, holdEnd);
    }

    public static OptimalPathV2 line(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createLineSupplier(
                drivePedroSubsystem,
                targetPose);

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    private static PathSupplier createLineSupplier(DrivePedroSubsystem dS, Pose targetPose){
        return builder -> {
            Pose currentDSPose = dS.getPose();
            BezierLine line = new BezierLine(currentDSPose, targetPose);
            double range = currentDSPose.distanceFrom(targetPose);

            HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingLine(
                    dS.getHeading(),
                    line,
                    targetPose.getHeading(),
                    range);

            return builder.addPath(
                    line)
                    .setHeadingInterpolation(headingInterpolator)
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();

        };
    }

    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingLine(
            double startHeading,
            BezierLine line,
            double targetHeading,
            double fullPathRange
    ){

        double forwardTangentAtLine = tangentHeadingAtLine(line);
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

        if (distanceForStartDelta + distanceForEndDelta < MIN_RANGE_FOR_PIECEWISE_INCHES) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(MIN_RANGE_FOR_PIECEWISE_INCHES));
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

    private static double tangentHeadingAtLine(BezierLine line) {
        return line.getEndTangent().getTheta();
    }

    public static OptimalPathV2 tri(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createCurveSupplier(
                drivePedroSubsystem,
                controlPoint1,
                targetPose);

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }
    public static OptimalPathV2 quad(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose, double maxPower, boolean holdEnd) {
        PathSupplier supplier = createCurveSupplier(
                drivePedroSubsystem,
                controlPoint1,
                controlPoint2,
                targetPose);

        return new OptimalPathV2(drivePedroSubsystem, supplier, maxPower, holdEnd);
    }

    private static PathSupplier createCurveSupplier(DrivePedroSubsystem dS, Pose controlPoint1, Pose targetPose){
        return builder -> {
            Pose currentDSPose = dS.getPose();
            BezierCurve curve = new BezierCurve(currentDSPose, controlPoint1, targetPose);
            double range = curve.approximateLength();

            HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    dS.getHeading(),
                    curve,
                    targetPose.getHeading(),
                    range);

            return builder.addPath(
                    curve)
                    .setHeadingInterpolation(headingInterpolator)
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();

        };
    }

    private static PathSupplier createCurveSupplier(DrivePedroSubsystem dS, Pose controlPoint1, Pose controlPoint2, Pose targetPose){
        return builder -> {
            Pose currentDSPose = dS.getPose();
            BezierCurve curve = new BezierCurve(currentDSPose, controlPoint1, controlPoint2, targetPose);
            double range = curve.approximateLength();

            HeadingInterpolator headingInterpolator = HeadingInterpolatorPieceWiseHeadingCurve(
                    dS.getHeading(),
                    curve,
                    targetPose.getHeading(),
                    range);

            return builder.addPath(
                            curve)
                    .setHeadingInterpolation(headingInterpolator)
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();

        };
    }


    private static HeadingInterpolator HeadingInterpolatorPieceWiseHeadingCurve(
            double startHeading,
            BezierCurve curve,
            double targetHeading,
            double fullPathRange
    ) {

        double forwardTangentAtBeginCurve = tangentHeadingAtTCurve(curve, 0);
        double reverseTangentAtBeginCurve = normalizeAngle(forwardTangentAtBeginCurve + Math.PI);

        double forwardTangentAtEndCurve = tangentHeadingAtTCurve(curve, 1);
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

        if (distanceForStartDelta + distanceForEndDelta < MIN_RANGE_FOR_PIECEWISE_INCHES) {
            return HeadingInterpolator.linear(startHeading, targetHeading, getLinearInterpolationHeadingEndTimeFromRange(fullPathRange));
        }

        double tTangentBegin = getTTangentFromRange(distanceForStartDelta, fullPathRange);
        double tangentBegin = tangentHeadingAtTCurve(curve, tTangentBegin);

        double tTangentEnd = 1 - getTTangentFromRange(distanceForEndDelta, fullPathRange);
        double tangentEnd = tangentHeadingAtTCurve(curve, tTangentEnd);

        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, tTangentBegin,
                        HeadingInterpolator.linear(startHeading, tangentBegin)),

                new HeadingInterpolator.PiecewiseNode(tangentBegin, tTangentEnd, forwardTangent ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()),

                new HeadingInterpolator.PiecewiseNode(tTangentEnd, 1,
                        HeadingInterpolator.linear(tangentEnd, targetHeading)));

    }


}