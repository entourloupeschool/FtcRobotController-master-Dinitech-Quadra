package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_FOR_3BALL;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAutoGate;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
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
                                        gatePower, true),
                                new SetVelocityShooter(shooterSubsystem, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),
                                new ParallelRaceGroup(
                                        new WaitCommand(WAIT_FOR_3BALL),
                                        new WaitUntilCommand(trieurSubsystem::getIsFull)),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                GatePickPose.withX(GatePickPose.getX() + (GatePickPose.getX() > 72 ? -1.5*TILE_DIM : 2*TILE_DIM)),
                                                endPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                endPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }
}
