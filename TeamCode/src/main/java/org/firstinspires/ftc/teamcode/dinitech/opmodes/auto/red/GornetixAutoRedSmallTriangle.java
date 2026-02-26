package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_SMALL_TRIANGLE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_RED_POSE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.InitToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ShootToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.GornetixAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

@Autonomous(name = "RedSmallTriangle - Dinitech", group = "Red")
public class GornetixAutoRedSmallTriangle extends GornetixAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);

            drivePedroSubsystem.getDrive().prepAuto(RED_SMALL_TRIANGLE_POSE);

            new SequentialCommandGroup(
                        // Obelisk and MoulinCalibrate
                        new InitToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, gamepadSubsystem,
                                RED_SMALL_TRIANGLE_SHOOT_POSE, SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY),

                        new ShootToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                THIRD_ROW_RED_POSE, RED_SMALL_TRIANGLE_SHOOT_POSE, new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem), SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY,
                                LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS, LINEAR_HEADING_INTERPOLATION_END_TIME/1.8),

                        new ShootToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                SECOND_ROW_RED_POSE, CLOSE_SHOOT_RED_POSE, new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY,
                                LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        new ShootToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                FIRST_ROW_RED_POSE, CLOSE_SHOOT_RED_POSE, new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY,
                                LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS, LINEAR_HEADING_INTERPOLATION_END_TIME/1.3),

                        new ParallelCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RED_RAMP_POSE.withX(RED_RAMP_POSE.getX() - 5))
                                        ).setLinearHeadingInterpolation(drivePedroSubsystem.getPose().getHeading(), RED_RAMP_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
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
