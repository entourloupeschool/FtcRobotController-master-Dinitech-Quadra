package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

//@Autonomous(name = "TestsXScale - Dinitech", group = "Test")
@Disabled

public class TestsXScale extends RobotBase {
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

            drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE, telemetryM);
            register(drivePedroSubsystem);


            trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
            register(trieurSubsystem);

            autoSetArtefactColors();

            chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
            register(chargeurSubsystem);

            shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
            register(shooterSubsystem);
            shooterSubsystem.setDefaultCommand(new VisionShooter(shooterSubsystem, visionSubsystem));


            new SequentialCommandGroup(
                    // Obelisk and MoulinCalibrate
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new MoulinCalibrationSequence(trieurSubsystem),
                                    new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem)
                            ),
                            new SequentialCommandGroup(
                                    new FollowPath(
                                            drivePedroSubsystem, builder -> builder
                                            .addPath(new BezierLine(
                                                    BEGIN_POSE,
                                                    BEGIN_POSE.withX(BEGIN_POSE.getX() + 2*TILE_DIM))
                                            ).build(),
                                            AUTO_ROBOT_CONSTRAINTS, true)
                            )
                    )
//                    new ParallelCommandGroup(
//                            new FollowPath(
//                                drivePedroSubsystem, builder -> builder
//                                .addPath(new BezierLine(
//                                        OBELISK_BLUE_POSE,
//                                        CLOSE_SHOOT_BLUE_POSE)
//                            ).setLinearHeadingInterpolation(OBELISK_BLUE_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
//                            AUTO_ROBOT_CONSTRAINTS, false),
//                            // Shoot All
//                            new ShootRevolution(trieurSubsystem, shooterSubsystem, new InstantRangeVisionShooter(shooterSubsystem, visionSubsystem))
//                    )


                    // go to next row of artefacts
//                    new ParallelRaceGroup(
//                            new ModeRamassage(trieurSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem),
//                            new SequentialCommandGroup(
//                                    new FollowPath(
//                                            drivePedroSubsystem, builder -> builder
//                                            .addPath(new BezierLine(
//                                                    CLOSE_SHOOT_BLUE_POSE,
//                                                    FIRST_ROW_BLUE_POSE)
//                                            ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), FIRST_ROW_BLUE_POSE.getHeading()).build(),
//                                            AUTO_ROBOT_CONSTRAINTS, true),
//                                    new MaxPowerChargeur(chargeurSubsystem),
//                                    new FollowPath(
//                                            drivePedroSubsystem, builder -> builder
//                                            .addPath(new BezierLine(
//                                                    FIRST_ROW_BLUE_POSE,
//                                                    FIRST_ROW_BLUE_POSE.withX(X_END_ROW_BLUE))
//                                            ).setLinearHeadingInterpolation(FIRST_ROW_BLUE_POSE.getHeading(), FIRST_ROW_BLUE_POSE.getHeading()).build(),
//                                            AUTO_ROBOT_CONSTRAINTS, true),
//                                    new StopChargeur(chargeurSubsystem)
//                            )
//                    )
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
