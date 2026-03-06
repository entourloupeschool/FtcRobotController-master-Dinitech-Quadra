package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUDIENCE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.unchained.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ThreeRowsFromGoal extends SequentialCommandGroup {

    public ThreeRowsFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem, double rowPower){
        addCommands(
                // Obelisk and MoulinCalibrate
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem),

                new SequentialCommandGroup(
                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                FIRST_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.7),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                SECOND_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                THIRD_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.3),

                        new ParallelCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.plus(new Pose(5, 0)) : RED_RAMP_POSE.plus(new Pose(- 5, 0))))
                                        .setLinearHeadingInterpolation(
                                                drivePedroSubsystem.getPose().getHeading(),
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.getHeading() : RED_RAMP_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new ParallelCommandGroup(
                                        new StopChargeur(chargeurSubsystem),
                                        new StopShooter(shooterSubsystem))))

        );
    }

    public ThreeRowsFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem, double rowPower, double shooterVelocity){
        addCommands(
                // Obelisk and MoulinCalibrate
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem, shooterVelocity),

                new SequentialCommandGroup(
                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                FIRST_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.7, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                SECOND_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                THIRD_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.3, AUDIENCE_AUTO_SHOOTER_VELOCITY),

                        new ParallelCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.plus(new Pose(5, 0)) : RED_RAMP_POSE.plus(new Pose(- 5, 0))))
                                        .setLinearHeadingInterpolation(
                                                drivePedroSubsystem.getPose().getHeading(),
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.getHeading() : RED_RAMP_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new ParallelCommandGroup(
                                        new StopChargeur(chargeurSubsystem),
                                        new StopShooter(shooterSubsystem))))

        );
    }
}
