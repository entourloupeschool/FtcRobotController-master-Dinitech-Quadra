package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

/**
 * Command to follow a path using PedroPathing.
 *
 * <p>Example usage with PathSupplier (recommended for inline paths):
 * <pre>
 * new FollowPath(driveSubsystem, builder -> builder
 *     .addPath(new BezierLine(
 *         new Point(0, 0, Point.CARTESIAN),
 *         new Point(24, 24, Point.CARTESIAN)
 *     ))
 *     .setLinearHeadingInterpolation(0, Math.PI / 2)
 *     .build(),
 *     0.8, true);
 * </pre>
 *
 * <p>Example usage with pre-built PathChain:
 * <pre>
 * PathChain myPath = driveSubsystem.getDrive().getPathBuilder()
 *     .addPath(...)
 *     .build();
 * new FollowPath(driveSubsystem, myPath, 0.8, true);
 * </pre>
 *
 * <p>Example usage with auto-configured target pose:
 * <pre>
 * new FollowPath(driveSubsystem, targetPose, 0.8, true);
 * </pre>
 */
public class FollowPath extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final PathSupplier pathSupplier;
    private PathChain pathChain;
    private final double maxPower;
    private final boolean holdEnd;
    private final Runnable onExecuteCallback;

    /**
     * Creates a FollowPath command using a PathSupplier (lambda-friendly).
     * The path is built at initialization time using the current robot pose.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathSupplier Lambda that receives a PathBuilder and returns a PathChain
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        this(drivePedroSubsystem, pathSupplier, maxPower, holdEnd, null);
    }

    /**
     * Creates a FollowPath command using a PathSupplier (lambda-friendly), with an
     * optional callback called every scheduler loop while the path is active.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathSupplier Lambda that receives a PathBuilder and returns a PathChain
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     * @param onExecuteCallback Optional callback run in execute(); may be null
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd, Runnable onExecuteCallback) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.pathSupplier = pathSupplier;
        this.pathChain = null;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        this.onExecuteCallback = onExecuteCallback;

        addRequirements(drivePedroSubsystem);
    }

    /**
     * Creates a FollowPath command with a pre-built PathChain.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathChain The pre-built PathChain to follow
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathChain pathChain, double maxPower, boolean holdEnd) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.pathSupplier = null;
        this.pathChain = pathChain;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        this.onExecuteCallback = null;

        addRequirements(drivePedroSubsystem);
    }

    /**
     * Creates a FollowPath command to drive directly to a target pose.
     *
     * <p>The path is built at initialization time from the robot's current pose to the target pose.
     * Heading interpolation end time and braking strength are scaled automatically from the current
     * range to the target using the project defaults in Globals.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param targetPose The end pose to drive to
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        this(drivePedroSubsystem, createDefaultBezierLineToTargetPoseSupplier(drivePedroSubsystem, targetPose), maxPower, holdEnd);
    }

    private static PathSupplier createDefaultBezierLineToTargetPoseSupplier(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose) {
        return builder -> {
            double range = drivePedroSubsystem.getPose().distanceFrom(targetPose);

            return builder
                    .addPath(new BezierLine(drivePedroSubsystem::getPose, targetPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            targetPose.getHeading(),
                            getLinearInterpolationHeadingEndTimeFromRange(range)))
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();
        };
    }

    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose, double maxPower, boolean holdEnd) {
        this(drivePedroSubsystem, createDefaultBezierCurveToTargetPoseSupplier(drivePedroSubsystem, controlPoint1, targetPose), maxPower, holdEnd);
    }
    private static PathSupplier createDefaultBezierCurveToTargetPoseSupplier(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose targetPose) {
        return builder -> {
            double range = drivePedroSubsystem.getPose().distanceFrom(targetPose);

            return builder
                    .addPath(new BezierCurve(drivePedroSubsystem::getPose, controlPoint1, targetPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            targetPose.getHeading(),
                            getLinearInterpolationHeadingEndTimeFromRange(range)))
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();
        };
    }

    private static PathSupplier createDefaultBezierCurveToTargetPoseSupplier(DrivePedroSubsystem drivePedroSubsystem, Pose controlPoint1, Pose controlPoint2, Pose targetPose) {
        return builder -> {
            double range = drivePedroSubsystem.getPose().distanceFrom(targetPose);

            return builder
                    .addPath(new BezierCurve(drivePedroSubsystem::getPose, controlPoint1, controlPoint2, targetPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            targetPose.getHeading(),
                            getLinearInterpolationHeadingEndTimeFromRange(range)))
                    .setBrakingStrength(getBrakingStrengthScaleFromRange(range))
                    .build();
        };
    }


    @Override
    public void initialize() {
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.NONE);

        // Build the path at initialization if using a supplier
        if (pathChain == null && pathSupplier != null) {
            pathChain = pathSupplier.build(drivePedroSubsystem.getDrive().getPathBuilder());
        }

        drivePedroSubsystem.followPathChain(pathChain, maxPower, holdEnd);
    }

    @Override
    public void execute() {
        if (onExecuteCallback != null) {
            onExecuteCallback.run();
        }
    }

    @Override
    public boolean isFinished() {
        return drivePedroSubsystem.isPathQuasiDone();
    }
}
