package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FollowTrajectory;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TeleDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleSlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.ToggleVisionTeleStopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAwayCondition;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootGreen;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootPurple;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "RosaceVisionTest - Dinitech", group = "Test")
public class RosaceVisionTest extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
        private VisionSubsystem visionSubsystem;
        private DriveSubsystem driveSubsystem;
    private DinitechMecanumDrive drive;

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
                visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));

                drive = new DinitechMecanumDrive(hardwareMap, visionSubsystem, BEGIN_POSE);

                driveSubsystem = new DriveSubsystem(drive, telemetry);
                register(driveSubsystem);

            Set<Subsystem> setDriveSubsystem = new HashSet<Subsystem>();
            setDriveSubsystem.add(driveSubsystem);

            new SequentialCommandGroup(
                    new FollowTrajectory(
                            RectangleCouleurs(drive, BEGIN_POSE), setDriveSubsystem
                    ),
                    new FollowTrajectory(
                            RectangleCouleurs(drive, BEGIN_POSE), setDriveSubsystem
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

    public static Action Rosace(DinitechMecanumDrive drive, Pose2d beginPose) {
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

    public static Action RectangleCouleurs(DinitechMecanumDrive drive, Pose2d beginPose) {
        double x_rectangle = 1.55*TILE_DIM;
        double y_rectangle = 1.34*TILE_DIM;

        Pose2d RectangleRouge = new Pose2d(x_rectangle, -y_rectangle, 1.05*Math.PI);
        Pose2d RectangleBleu = new Pose2d(x_rectangle, y_rectangle, 1.25*Math.PI);
        Pose2d OpposeRectangleRouge = new Pose2d(-x_rectangle, -y_rectangle, 1.2*Math.PI);
        Pose2d OpposeRectangleBleu = new Pose2d(-x_rectangle, y_rectangle, 1.45*Math.PI);
        TrajectoryActionBuilder builder = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(RectangleRouge.position, RectangleRouge.heading)
                .strafeToLinearHeading(RectangleBleu.position, RectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleBleu.position, OpposeRectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleRouge.position, OpposeRectangleRouge.heading)
                .strafeToLinearHeading(new Vector2d(0, 0), Math.PI);


        return builder.build();
    }

}
