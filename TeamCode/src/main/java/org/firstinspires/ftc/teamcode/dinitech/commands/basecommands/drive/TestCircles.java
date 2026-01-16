package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BACKWARD_CIRCLE_CENTER_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FORWARD_CIRCLE_CENTER_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LEFT_CIRCLE_CENTER_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RIGHT_CIRCLE_CENTER_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TEST_CIRCLE_RADIUS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TEST_CONSTRAINTS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;

/**
 * A command for testing and tuning the drive base by following a complex series of circular paths.
 * <p>
 * This command extends {@link FollowTrajectory} and builds a single, long trajectory composed
 * of four distinct circles (left, right, forward, and backward). It is designed to test the
 * robot's ability to follow splines and control its heading accurately. The trajectory is built
 * using Road Runner's {@code actionBuilder} and is likely used for debugging, tuning PID
 * controllers, or verifying the drive base kinematics.
 */
public class TestCircles extends FollowTrajectory {

    /**
     * Creates a new TestCircles command.
     *
     * @param driveSubsystem The drive subsystem to control.
     */
    public TestCircles(DriveSubsystem driveSubsystem){
        super(
                // Build a complex trajectory that traces four circles.
                driveSubsystem.getDrive().actionBuilder(driveSubsystem.getPose(), TEST_CONSTRAINTS)
                        // Left circle
                        .splineToLinearHeading(LEFT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(TEST_CIRCLE_RADIUS,0), -Math.PI/2)), Math.PI/2)
                        .splineToLinearHeading(LEFT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,TEST_CIRCLE_RADIUS), -Math.PI)), Math.PI)
                        .splineToLinearHeading(LEFT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(-TEST_CIRCLE_RADIUS,0), Math.PI/2)), -Math.PI/2)
                        .splineToLinearHeading(LEFT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,-TEST_CIRCLE_RADIUS), 0)), 0)

                        // Right circle
                        .splineToLinearHeading(RIGHT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(TEST_CIRCLE_RADIUS,0), Math.PI/2)), -Math.PI/2)
                        .splineToLinearHeading(RIGHT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,-TEST_CIRCLE_RADIUS), Math.PI)), Math.PI)
                        .splineToLinearHeading(RIGHT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(-TEST_CIRCLE_RADIUS,0), -Math.PI/2)), Math.PI/2)
                        .splineToLinearHeading(RIGHT_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,TEST_CIRCLE_RADIUS), 0)), 0)

                        .setTangent(-Math.PI/2)
                        // Forward circle
                        .splineToLinearHeading(FORWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,-TEST_CIRCLE_RADIUS), -Math.PI/2)), 0)
                        .splineToLinearHeading(FORWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(TEST_CIRCLE_RADIUS,0), -Math.PI)), Math.PI/2)
                        .splineToLinearHeading(FORWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0,TEST_CIRCLE_RADIUS), Math.PI/2)), -Math.PI)
                        .splineToLinearHeading(FORWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(-TEST_CIRCLE_RADIUS,0), 0)), -Math.PI/2)

                        // Backward circle
                        .splineToLinearHeading(BACKWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0, -TEST_CIRCLE_RADIUS), Math.PI/2)), -Math.PI)
                        .splineToLinearHeading(BACKWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(-TEST_CIRCLE_RADIUS,0), -Math.PI)), Math.PI/2)
                        .splineToLinearHeading(BACKWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(0, TEST_CIRCLE_RADIUS), -Math.PI/2)), 0)
                        .splineToLinearHeading(BACKWARD_CIRCLE_CENTER_POSE.times(new Pose2d(new Vector2d(TEST_CIRCLE_RADIUS,0), 0)), -Math.PI/2)

                        .build(), driveSubsystem
        );
    }
}
