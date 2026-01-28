package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.END_GAME_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdatesAprilTagsDetections;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAlmostRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassage;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

@Autonomous(name = "GornetixAutoBlueGoal - Dinitech", group = "Auto")
public class GornetixAutoBlueGoal extends DinitechRobotBase {
    private TrieurSubsystem trieurSubsystem;
    private VisionSubsystem visionSubsystem;
    private GamepadSubsystem gamepadSubsystem;


    private ShooterSubsystem shooterSubsystem;
    private ChargeurSubsystem chargeurSubsystem;
    private DrivePedroSubsystem drivePedroSubsystem;


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);

            visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
            register(visionSubsystem);
            visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

            drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BLUE_GOAL_POSE, telemetryM);
            register(drivePedroSubsystem);

            trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
            register(trieurSubsystem);

            autoSetArtefactColors();

            chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
            register(chargeurSubsystem);

            shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
            register(shooterSubsystem);


            new SequentialCommandGroup(
                    // Obelisk and MoulinCalibrate
                    new ParallelCommandGroup(
                            new SetVelocityShooter(shooterSubsystem, CLOSE_SHOOT_BLUE_POSE_SHOOTER_VELOCITY),
                            new SequentialCommandGroup(
                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new CloseTrappe(trieurSubsystem),
                                                    new WaitCommand(200)),
                                            new InstantCommand(),
                                            trieurSubsystem::isTrappeOpen
                                    ),
                                    new MoulinCalibrate(trieurSubsystem),
                                    new InstantCommand(() -> trieurSubsystem.hardSetMoulinPosition(6), trieurSubsystem)
                            ),
                            // Go to Shooting Pos
                            new FollowPath(drivePedroSubsystem, builder -> builder
                                    .addPath(new BezierLine(
                                            drivePedroSubsystem::getPose,
                                            CLOSE_SHOOT_BLUE_POSE)
                                    ).setLinearHeadingInterpolation(BLUE_GOAL_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                    AUTO_ROBOT_CONSTRAINTS, true)),


                    new ShootAlmostRevolution(trieurSubsystem, new InstantCommand()),

                    new ParallelCommandGroup(
                            new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem).withTimeout(MODE_RAMASSAGE_TIMEOUT),
                            new SequentialCommandGroup(
                                    // go to first row of artefacts
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    FIRST_ROW_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), FIRST_ROW_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true),
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    FIRST_ROW_BLUE_POSE.withX(FIRST_ROW_BLUE_POSE.getX() - LENGTH_X_ROW))
                                            ).setConstantHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading()).build(),
                                            MAX_POWER_ROW_PICK_ARTEFACTS, true),
                                    // Go to Shooting Pos
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    CLOSE_SHOOT_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true))),

                    new ShootAlmostRevolution(trieurSubsystem, new InstantCommand()),

                    new ParallelCommandGroup(
                            new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem).withTimeout(MODE_RAMASSAGE_TIMEOUT),
                            new SequentialCommandGroup(
                                    // go to second row of artefacts
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    SECOND_ROW_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), SECOND_ROW_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true),
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    SECOND_ROW_BLUE_POSE.withX(SECOND_ROW_BLUE_POSE.getX() - LENGTH_X_ROW))
                                            ).setConstantHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading()).build(),
                                            MAX_POWER_ROW_PICK_ARTEFACTS, true),
                                    // Go to Shooting Pos
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    CLOSE_SHOOT_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true))),

                    new ShootAlmostRevolution(trieurSubsystem, new InstantCommand()),

                    new ParallelCommandGroup(
                            new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem).withTimeout(MODE_RAMASSAGE_TIMEOUT),
                            new SequentialCommandGroup(
                                    // go to third row of artefacts
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    THIRD_ROW_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), THIRD_ROW_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true),
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    THIRD_ROW_BLUE_POSE.withX(THIRD_ROW_BLUE_POSE.getX() - LENGTH_X_ROW))
                                            ).setConstantHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading()).build(),
                                            MAX_POWER_ROW_PICK_ARTEFACTS, true),
                                    // Go to Shooting Pos
                                    new FollowPath(drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    drivePedroSubsystem::getPose,
                                                    CLOSE_SHOOT_BLUE_POSE)
                                            ).setLinearHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true))),

                    new ShootAlmostRevolution(trieurSubsystem, new InstantCommand()),

                    // Go to end
                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierLine(
                                    drivePedroSubsystem::getPose,
                                    END_GAME_RED_POSE)
                            ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), END_GAME_RED_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                            AUTO_ROBOT_CONSTRAINTS, true)
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

}
