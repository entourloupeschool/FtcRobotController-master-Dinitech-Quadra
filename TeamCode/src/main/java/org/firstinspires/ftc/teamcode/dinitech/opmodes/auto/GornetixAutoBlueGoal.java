package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_ARTEFACTS_PREP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_ARTEFACT_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OBELISK_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_ARTEFACTS_PREP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_ARTEFACTS_PREP_POSE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FollowTrajectory;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotifAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassage;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "GornetixAutoBlueGoal - Dinitech", group = "Auto")
public class GornetixAutoBlueGoal extends DinitechRobotBase {
    private TrieurSubsystem trieurSubsystem;
    private VisionSubsystem visionSubsystem;
    private GamepadSubsystem gamepadSubsystem;


    private ShooterSubsystem shooterSubsystem;
    private ChargeurSubsystem chargeurSubsystem;
    private DinitechMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private final Set<Subsystem> setDriveSubsystem = new HashSet<Subsystem>();


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);

            visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
            register(visionSubsystem);
            visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));

            drive = new DinitechMecanumDrive(hardwareMap, visionSubsystem, BEGIN_POSE);
            driveSubsystem = new DriveSubsystem(drive, telemetry);
            register(driveSubsystem);

            setDriveSubsystem.add(driveSubsystem);

            trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
            register(trieurSubsystem);

            autoSetArtefactColors();

            chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetry);
            register(chargeurSubsystem);

            shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
            register(shooterSubsystem);


            // LISTENER: Automatic trigger: when trieur becomes full, spin up shooter to max speed and
            new Trigger(trieurSubsystem::getIsFull)
                    .whenActive(new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem));

            new SequentialCommandGroup(
                    // Obelisk and MoulinCalibrate
                    new ParallelCommandGroup(
                            new MoulinCalibrate(trieurSubsystem),
                            new FollowTrajectory(
                                    drive.actionBuilder(drive.localizer.getPose(), AUTO_ROBOT_CONSTRAINTS)
                                            .strafeToLinearHeading(OBELISK_POSE.position, OBELISK_POSE.heading)
                                            .build(), setDriveSubsystem)),

                    // Shoot close - just rotate since we're already at the right position
                    new ParallelCommandGroup(
                            new FollowTrajectory(
                                    drive.actionBuilder(drive.localizer.getPose(), AUTO_ROBOT_CONSTRAINTS)
                                            .turnTo(CLOSE_SHOOT_POSE.heading)
                                            .build(), setDriveSubsystem),
                            new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem)),

                    // Shoot All
                    new ShootRevolution(trieurSubsystem, shooterSubsystem, new VisionShooter(shooterSubsystem, visionSubsystem)),

                    // go to next row of artefacts
                    new ParallelCommandGroup(
                            new ModeRamassage(trieurSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem),
                            new FollowTrajectory(
                                    drive.actionBuilder(drive.localizer.getPose(), AUTO_ROBOT_CONSTRAINTS)
                                            .strafeToLinearHeading(FIRST_ROW_ARTEFACTS_PREP_POSE.position, FIRST_ROW_ARTEFACTS_PREP_POSE.heading)
                                            .strafeToConstantHeading(new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x, FIRST_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))
                                            .build(), setDriveSubsystem))
            ).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }

    /**
     * auto set artefact colors
     */
    private void autoSetArtefactColors(){
            trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setMoulinStoragePositionColor(2, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.GREEN);
    }

    public static Action BlueFromBasket(DinitechMecanumDrive drive, Pose2d beginPose) {
        return drive.actionBuilder(beginPose, AUTO_ROBOT_CONSTRAINTS)
                // See the obelisk
                .strafeToLinearHeading(OBELISK_POSE.position, OBELISK_POSE.heading)

                // Trieur busines

                // Shoot close - just rotate since we're already at the right position
                .turnTo(CLOSE_SHOOT_POSE.heading)

                // go to next row of artefacts
                .strafeToLinearHeading(FIRST_ROW_ARTEFACTS_PREP_POSE.position, FIRST_ROW_ARTEFACTS_PREP_POSE.heading)
                .strafeToConstantHeading(new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x, FIRST_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                // Shoot close
                .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)

                // go to next row of artefacts
                .strafeToLinearHeading(SECOND_ROW_ARTEFACTS_PREP_POSE.position, SECOND_ROW_ARTEFACTS_PREP_POSE.heading)
                .strafeToConstantHeading(new Vector2d(SECOND_ROW_ARTEFACTS_PREP_POSE.position.x, SECOND_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                // Shoot close
                .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)

                // go to next row of artefacts
                .strafeToLinearHeading(THIRD_ROW_ARTEFACTS_PREP_POSE.position, THIRD_ROW_ARTEFACTS_PREP_POSE.heading)
                .strafeToConstantHeading(new Vector2d(THIRD_ROW_ARTEFACTS_PREP_POSE.position.x, THIRD_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                // Shoot close
                .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)
                .build();
    }

}
