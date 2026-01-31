package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;


import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.InitToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ShootToRowToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.GornetixAutoBase;

@Autonomous(name = "BlueGoal - Dinitech", group = "Blue")
public class GornetixAutoBlueGoal extends GornetixAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.getDrive().prepAuto(BLUE_GOAL_POSE);

            new SequentialCommandGroup(
                    // Obelisk and MoulinCalibrate
                    new InitToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            BLUE_GOAL_POSE, CLOSE_SHOOT_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            CLOSE_SHOOT_BLUE_POSE, FIRST_ROW_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            CLOSE_SHOOT_BLUE_POSE, SECOND_ROW_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),

                    new ShootToRowToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                            CLOSE_SHOOT_BLUE_POSE, THIRD_ROW_BLUE_POSE, BLUE_SMALL_TRIANGLE_SHOOT_POSE, SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY, LENGTH_X_ROW, MAX_POWER_ROW_PICK_ARTEFACTS),

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


}
