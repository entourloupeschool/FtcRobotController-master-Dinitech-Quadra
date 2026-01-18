package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FollowTrajectory;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FollowTrajectoryOld;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "FollowTrajectoryVisionTest - Dinitech", group = "Test")
public class FollowTrajectoryVisionTest extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private VisionSubsystem visionSubsystem;
    private DriveSubsystem driveSubsystem;
    private Set<Subsystem> setRequirements = new HashSet<>();



    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        register(visionSubsystem);

        driveSubsystem = new DriveSubsystem(hardwareMap, BEGIN_POSE, telemetry);
        register(driveSubsystem);

        setRequirements.add(driveSubsystem);

        new SequentialCommandGroup(
                new FollowTrajectoryOld(
                        RectangleCouleurs(driveSubsystem.getDrive()), setRequirements
                ),
                new FollowTrajectory(
                        RectangleCouleurs(driveSubsystem.getDrive()), driveSubsystem
                ),
                new FollowTrajectory(
                        RectangleCouleurs(driveSubsystem.getDrive()), driveSubsystem, visionSubsystem
                )
        ).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }

    public static Action Rosace(DinitechMecanumDrive drive) {
        Pose2d beginPose = drive.localizer.getPose();
        TrajectoryActionBuilder builder = drive.actionBuilder(beginPose, AUTO_ROBOT_CONSTRAINTS);

        double botLengthCM = 38.5;
        double cmToInch = 1.0 / 2.54;

        double Rosace_length = 2 * botLengthCM * cmToInch;

        double tranchePI = 4;
        for (int i=1; i < tranchePI*2; i++){
            double angleOffset = i * Math.PI / tranchePI;

            //Calculate angles for the Left and Right petals
            double leftAngle = -Math.PI / 2 + angleOffset;
            double rightAngle = Math.PI / 2 + angleOffset;

            //Define the Poses for the tips of the petals
            Pose2d leftPetalTip = new Pose2d(
                    new Vector2d(
                            beginPose.position.x + Rosace_length * Math.cos(leftAngle),
                            beginPose.position.y + Rosace_length * Math.sin(leftAngle)
                    ),
                    beginPose.heading // Face the direction of the petal
            );
            Pose2d rightPetalTip = new Pose2d(
                    new Vector2d(
                            beginPose.position.x + Rosace_length * Math.cos(rightAngle),
                            beginPose.position.y + Rosace_length * Math.sin(rightAngle)
                    ),
                    beginPose.heading // Face the direction of the petal
            );

            builder = builder.setTangent(angleOffset)
                    .splineToLinearHeading(leftPetalTip, angleOffset)
                    .splineToLinearHeading(beginPose, angleOffset)
                    .splineToLinearHeading(rightPetalTip, angleOffset)
                    .splineToLinearHeading(beginPose, angleOffset);
        }
        return builder.build();
    }

    public static Action RectangleCouleurs(DinitechMecanumDrive drive) {
        Pose2d beginPose = drive.localizer.getPose();
        double x_rectangle = 1.55*TILE_DIM;
        double y_rectangle = 1.34*TILE_DIM;

        // Use the starting heading (0) for all waypoints
        // strafeToLinearHeading with large heading changes (like 1.05*PI from 0) 
        // causes the robot to spend most time rotating instead of moving
        double heading = beginPose.heading.toDouble(); // Should be 0
        
        Vector2d RectangleRouge = new Vector2d(x_rectangle, -y_rectangle);
        Vector2d RectangleBleu = new Vector2d(x_rectangle, y_rectangle);
        Vector2d OpposeRectangleRouge = new Vector2d(-x_rectangle, -y_rectangle);
        Vector2d OpposeRectangleBleu = new Vector2d(-x_rectangle, y_rectangle);
        
        TrajectoryActionBuilder builder = drive.actionBuilder(beginPose, AUTO_ROBOT_CONSTRAINTS)
                // Using strafeToLinearHeading with same heading = effectively strafeTo
                .strafeToLinearHeading(RectangleRouge, heading)
                .turnTo(2*Math.PI + 1e-6)
                .strafeToLinearHeading(RectangleBleu, heading)
                .turnTo(2*Math.PI + 1e-6)
                .strafeToLinearHeading(OpposeRectangleBleu, heading)
                .turnTo(2*Math.PI + 1e-6)
                .strafeToLinearHeading(OpposeRectangleRouge, heading)
                .turnTo(2*Math.PI + 1e-6)
                .strafeToLinearHeading(new Vector2d(0, 0), heading);

        return builder.build();
    }

}
