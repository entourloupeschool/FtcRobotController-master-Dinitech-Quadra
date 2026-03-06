package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.T_PARAMETRIC_DONT_SHOOT;

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
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToRowToShoot extends SequentialCommandGroup {
    public ToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose RowPose, Pose endPose, double lengthBackup, double rowPower, double endTime, double shooterVeolocity){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        RowPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        RowPose.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? lengthBackup : -lengthBackup))))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                RowPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                        rowPower, true),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVeolocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                RowPose,
                                                endPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                endPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.getHowManyArtefacts() == 0) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)),
                        new ModeRamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                        new MaxPowerChargeur(chargeurSubsystem)),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }

    public ToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose RowPose, Pose endPose, double lengthBackup, double endTime, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new TrieurReadyEmptyStorage(trieurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem)),
                        new MaxPowerChargeur(chargeurSubsystem),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPaths(
                                                new BezierLine(
                                                        drivePedroSubsystem::getPose,
                                                        RowPose),
                                                new BezierLine(
                                                        drivePedroSubsystem::getPose,
                                                        RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? lengthBackup : -lengthBackup))))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                RowPose.getHeading(),
                                                endTime)).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                RowPose,
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
