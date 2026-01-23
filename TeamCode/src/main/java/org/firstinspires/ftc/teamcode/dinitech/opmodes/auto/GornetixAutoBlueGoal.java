package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;


import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.InstantRangeVisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdatesAprilTagsDetections;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassage;
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
            shooterSubsystem.setDefaultCommand(new VisionShooter(shooterSubsystem, visionSubsystem));


            new SequentialCommandGroup(
                    // Obelisk and MoulinCalibrate
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new MoulinCalibrationSequence(trieurSubsystem),
                                    new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem)

                            ),
                            new FollowPath(
                                    drivePedroSubsystem, builder -> builder
                                    .addPath(new BezierLine(
                                            BLUE_GOAL_POSE,
                                            CLOSE_SHOOT_BLUE_POSE)
                                    ).setLinearHeadingInterpolation(BLUE_GOAL_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                    AUTO_ROBOT_CONSTRAINTS, false)
                    ),

                    // Shoot All
                    new ShootRevolution(trieurSubsystem, shooterSubsystem, new InstantRangeVisionShooter(shooterSubsystem, visionSubsystem))

                    // go to next row of artefacts
//                    new ParallelCommandGroup(
//                            new ModeRamassage(trieurSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem),
//                            new FollowPath(
//                                    drivePedroSubsystem, builder -> builder
//                                    .addPath(new BezierLine(
//                                            OBELISK_POSE,
//                                            CLOSE_SHOOT_BLUE_POSE)
//                                    ).setLinearHeadingInterpolation(OBELISK_POSE.getHeading(), CLOSE_SHOOT_BLUE_POSE.getHeading()).build(),
//                                    AUTO_ROBOT_CONSTRAINTS, true
//                            )
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
