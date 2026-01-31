package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_AT_END_ROW;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.InitToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ShootToRowToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootTimeAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

@Autonomous(name = "BlueSmallTriangle - Dinitech", group = "Blue")
public class GornetixAutoBlueSmallTriangle extends DinitechRobotBase {
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

            drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BLUE_SMALL_TRIANGLE_POSE, telemetryM);
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
                    new InitToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            BLUE_SMALL_TRIANGLE_POSE, BLUE_SMALL_TRIANGLE_SHOOT_POSE, SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            BLUE_SMALL_TRIANGLE_SHOOT_POSE, THIRD_ROW_BLUE_POSE, SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            BLUE_SMALL_TRIANGLE_SHOOT_POSE, SECOND_ROW_BLUE_POSE, CLOSE_SHOOT_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            CLOSE_SHOOT_BLUE_POSE, FIRST_ROW_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),


                    new ParallelCommandGroup(
                            new FollowPath(drivePedroSubsystem, builder -> builder
                                    .addPath(new BezierLine(
                                            drivePedroSubsystem::getPose,
                                            BLUE_RAMP_POSE.withX(BLUE_RAMP_POSE.getX() + 5))
                                    ).setLinearHeadingInterpolation(CLOSE_SHOOT_BLUE_POSE.getHeading(), BLUE_RAMP_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                    AUTO_ROBOT_CONSTRAINTS, true),
                            new ParallelCommandGroup(
                                    new StopChargeur(chargeurSubsystem),
                                    new StopShooter(shooterSubsystem)))




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
        trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.GREEN);
        trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.PURPLE);
        trieurSubsystem.setMoulinStoragePositionColor(5, TrieurSubsystem.ArtifactColor.PURPLE);
    }

}
