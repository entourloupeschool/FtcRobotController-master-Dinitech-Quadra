package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.transformToPedroCoordinates;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FollowTrajectory;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

@Autonomous(name = "FollowTrajectoryPedroTest - Dinitech", group = "Test")
public class FollowTrajectoryPedroTest extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private VisionSubsystem visionSubsystem;
    private DrivePedroSubsystem drivePedroSubsystem;


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE, telemetryM);
        register(drivePedroSubsystem);

        new SequentialCommandGroup(
                new FollowPath(drivePedroSubsystem,
                        new PathChain(
                                new Path(
                                        new BezierLine(
                                                BEGIN_POSE,
                                                BEGIN_POSE.withX(BEGIN_POSE.getX() + 20)
                                        )
                                ),
                                new Path(
                                        new BezierLine(
                                                drivePedroSubsystem.getPose(),
                                                BEGIN_POSE
                                        )
                                )),
                        AUTO_ROBOT_CONSTRAINTS, true),
                new WaitCommand(1000),
                new FollowPath(drivePedroSubsystem,
                        new PathChain(
                                new Path(
                                        new BezierLine(
                                                BEGIN_POSE,
                                                BEGIN_POSE.withX(BEGIN_POSE.getX() + 20)
                                        )
                                ),
                                new Path(
                                        new BezierLine(
                                                drivePedroSubsystem.getPose(),
                                                BEGIN_POSE
                                        )
                                )),
                        AUTO_ROBOT_CONSTRAINTS, true)
        ).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
            telemetry.addData("x", drivePedroSubsystem.getPose().getX());
            telemetry.addData("y", drivePedroSubsystem.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(drivePedroSubsystem.getPose().getHeading()));
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
//        double heading = beginPose.heading.toDouble(); // Should be 0

        Pose2d RectangleRouge = new Pose2d(x_rectangle, -y_rectangle, 1.05*Math.PI);
        Pose2d RectangleBleu = new Pose2d(x_rectangle, y_rectangle, 1.25*Math.PI);
        Pose2d OpposeRectangleRouge = new Pose2d(-x_rectangle, -y_rectangle, 1.2*Math.PI);
        Pose2d OpposeRectangleBleu = new Pose2d(-x_rectangle, y_rectangle, 1.45*Math.PI);
        
        TrajectoryActionBuilder builder = drive.actionBuilder(beginPose, AUTO_ROBOT_CONSTRAINTS)
                // Using strafeToLinearHeading with same heading = effectively strafeTo
                .strafeToLinearHeading(RectangleRouge.position, RectangleRouge.heading)
                .strafeToLinearHeading(RectangleBleu.position, RectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleBleu.position, OpposeRectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleRouge.position, OpposeRectangleRouge.heading)
                .strafeToLinearHeading(new Pose2d(new Vector2d(0, 0), 0).position, Math.PI);

        return builder.build();
    }

}
