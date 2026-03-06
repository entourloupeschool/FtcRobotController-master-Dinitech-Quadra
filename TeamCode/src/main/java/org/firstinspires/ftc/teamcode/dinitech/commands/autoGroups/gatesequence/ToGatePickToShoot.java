package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUDIENCE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_FOR_3BALL;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAutoGate;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToGatePickToShoot extends SequentialCommandGroup {
    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose GatePickPose, Pose GatePickEndPose, Pose endPose, double gatePower, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierCurve(
                                        drivePedroSubsystem::getPose,
                                        GatePickPose.withX(GatePickPose.getX() + (GatePickPose.getX() > 72 ? -1.5*TILE_DIM : 1.5*TILE_DIM)),
                                        GatePickPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        GatePickPose.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new MaxPowerChargeur(chargeurSubsystem),
                        new ModeRamassageAutoGate(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                GatePickEndPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                GatePickEndPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        gatePower, false),
                                new SetVelocityShooter(shooterSubsystem, endPose.getY() > 72 ? CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY : AUDIENCE_AUTO_SHOOTER_VELOCITY),
                                new WaitCommand(WAIT_FOR_3BALL),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                GatePickPose.withX(GatePickPose.getX() + (GatePickPose.getX() > 72 ? -1.5*TILE_DIM : 2*TILE_DIM)),
                                                endPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                endPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }

    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem, double gatePower, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierCurve(
                                        drivePedroSubsystem::getPose,
                                        hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.plus(new Pose(1.5*TILE_DIM, 0)) : RED_RAMP_POSE.plus(new Pose(-1.5*TILE_DIM, 0)),
                                        hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE : RED_RAMP_POSE))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.getHeading() : RED_RAMP_POSE.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new ModeRamassageAutoGate(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_END_POSE : RED_RAMP_END_POSE))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_END_POSE.getHeading() : RED_RAMP_END_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        gatePower, false),
                                new SetVelocityShooterRequire(shooterSubsystem, AUDIENCE_AUTO_SHOOTER_VELOCITY),
                                new WaitCommand(WAIT_FOR_3BALL),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.plus(new Pose(1.5*TILE_DIM, 0)) : RED_RAMP_POSE.plus(new Pose(-1.5*TILE_DIM, 0)),
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? CLOSE_SHOOT_BLUE_POSE : CLOSE_SHOOT_RED_POSE))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? CLOSE_SHOOT_BLUE_POSE.getHeading() : CLOSE_SHOOT_RED_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(0.75, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }
}
