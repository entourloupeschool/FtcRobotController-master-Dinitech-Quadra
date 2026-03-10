package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.T_PARAMETRIC_DONT_SHOOT;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToRowToShoot extends SequentialCommandGroup {

    public ToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose rowPose, Pose shootPose, double lengthBackup, double rowPower, double endTime, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        rowPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        rowPose.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? lengthBackup : -lengthBackup))))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                rowPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        rowPower, true),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? -5 : 5)),
                                                shootPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                shootPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)),
                        new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem)),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }

    public ToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose rowPose, Pose shootPose, CommandBase openGateTrajectoryCommand, double lengthBackup, double rowPower, double endTime, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        rowPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        rowPose.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? lengthBackup : -lengthBackup))))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                rowPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        rowPower, true),
                                openGateTrajectoryCommand,
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? -5 : 5)),
                                                shootPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                shootPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)),
                        new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem)),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }

    public ToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose rowPose, Pose shootPose, double lengthBackup, double endTime, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new TrieurReadyEmptyStorage(trieurSubsystem),
                                new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem)),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPaths(
                                                new BezierLine(
                                                        drivePedroSubsystem::getPose,
                                                        rowPose),
                                                new BezierLine(
                                                        drivePedroSubsystem::getPose,
                                                        rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? lengthBackup : -lengthBackup))))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                rowPose.getHeading(),
                                                endTime)).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                rowPose,
                                                shootPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                shootPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }
}
