package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToWallToShoot extends SequentialCommandGroup {

    public ToWallToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose wallPickPose, Pose shootPose, double lengthBackup, double rowPower, double endTime, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MoulinCalibrationSequence(trieurSubsystem),
                                new TrieurReadyEmptyStorage(trieurSubsystem)),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        wallPickPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        wallPickPose.getHeading(),
                                        endTime)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new RamassageAuto(trieurSubsystem, visionSubsystem, chargeurSubsystem, false),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new FollowPath(drivePedroSubsystem, builder -> builder
                                                        .addPath(new BezierLine(
                                                                drivePedroSubsystem::getPose,
                                                                wallPickPose.withY(wallPickPose.getY() + lengthBackup)))
                                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                                drivePedroSubsystem::getHeading,
                                                                wallPickPose.getHeading(),
                                                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                                        rowPower, true),
                                                new WaitCommand(2000)),
                                        new WaitUntilCommand(trieurSubsystem::isFull)),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                shootPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                shootPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.isEmpty()) this.cancel();}).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootAll(trieurSubsystem, shooterSubsystem, true)
        );
    }
}
