package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getLinearInterpolationHeadingEndTimeFromRange;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

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
public class LineLinearHeadingInterpolationToTarget extends FollowPath {

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
    public LineLinearHeadingInterpolationToTarget(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose, double maxPower, boolean holdEnd) {
        super(drivePedroSubsystem, createDefaultBezierLineToTargetPoseLinearHeadingInterpolationSupplier(drivePedroSubsystem, targetPose), maxPower, holdEnd);
    }


    private static PathSupplier createDefaultBezierLineToTargetPoseLinearHeadingInterpolationSupplier(DrivePedroSubsystem drivePedroSubsystem, Pose targetPose) {
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

}
