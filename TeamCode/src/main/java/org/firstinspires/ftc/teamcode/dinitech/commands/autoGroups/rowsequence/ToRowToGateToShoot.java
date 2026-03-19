package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_STRENGTH_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.UNSHORTCUT_LENGTH;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToRowToGateToShoot extends SequentialCommandGroup {
    public ToRowToGateToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose rowPose, Pose shootPose, Pose openRampPose, double lengthBackup, double rowPower, double endTime, double shooterVelocity, boolean shortcutBackPath, double scaleBrakingStrength){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MoulinCalibrationSequence(trieurSubsystem),
                                new TrieurReadyEmptyStorage(trieurSubsystem)),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        rowPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        rowPose.getHeading(),
                                        endTime))
                                .setBrakingStrength(scaleBrakingStrength).build(),
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
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierCurve(
                                                drivePedroSubsystem::getPose,
                                                openRampPose
                                                        .withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -TILE_DIM/1.5 : TILE_DIM/1.5))
                                                        .withY(openRampPose.getY() -4),
                                                openRampPose
                                                        .withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -TILE_DIM/1.5 : TILE_DIM/1.5))
                                                        .withY(openRampPose.getY() +  4),
                                                openRampPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                openRampPose.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT))
                                        .setBrakingStrength(scaleBrakingStrength).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new WaitCommand(2000),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(shortcutBackPath ?
                                                new BezierLine(
                                                        drivePedroSubsystem::getPose,
                                                        shootPose)
                                                : new BezierCurve(
                                                        drivePedroSubsystem::getPose,
                                                        rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? -UNSHORTCUT_LENGTH : UNSHORTCUT_LENGTH)),
                                                        shootPose))
                                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                                drivePedroSubsystem::getHeading,
                                                shootPose.getHeading(),
                                                endTime))
                                        .addParametricCallback(T_PARAMETRIC_DONT_SHOOT, () -> {
                                            if (trieurSubsystem.isEmpty()) this.cancel();})
                                        .setBrakingStrength(scaleBrakingStrength).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)),
                        new RamassageAuto(trieurSubsystem, visionSubsystem, chargeurSubsystem, false)),

                new ShootAll(trieurSubsystem, shooterSubsystem, true)
        );
    }
}
