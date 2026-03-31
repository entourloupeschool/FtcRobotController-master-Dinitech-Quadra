package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.UNSHORTCUT_LENGTH;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToRowToGateToShoot extends SequentialCommandGroup {
    public ToRowToGateToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, HubsSubsystem hubsSubsystem, Pose rowPose, Pose shootPose, Pose openRampPose, double lengthBackup, double rowPower, boolean shortcutBackPath){
        double backupForGatePush = openRampPose.getX() + (openRampPose.getX() > 72 ? -TILE_DIM / 1.5 : TILE_DIM / 1.5);
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.line(drivePedroSubsystem,
                                rowPose, 1, true)),

                new ParallelCommandGroup(
                        new RamassageAuto(trieurSubsystem, visionSubsystem, chargeurSubsystem, false),
                        new SequentialCommandGroup(
                                OptimalPath.line(drivePedroSubsystem,
                                        rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? lengthBackup : -lengthBackup)), rowPower, true),
                                OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose
                                                .withX(backupForGatePush)
                                                .withY(openRampPose.getY() -4),
                                        openRampPose
                                                .withX(backupForGatePush)
                                                .withY(openRampPose.getY() +  4),
                                        openRampPose, 1, true),

                                new WaitCommand(1000),
                                new SetVelocityShooterRequire(shooterSubsystem,
                                        ShooterSubsystem.linearSpeedFromPedroRange(
                                                shootPose.distanceFrom(hubsSubsystem.getTeam().getBasketPose()))),
                                shortcutBackPath ?
                                        OptimalPath.line(drivePedroSubsystem,
                                                shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                                        OptimalPath.curve(drivePedroSubsystem,
                                                rowPose.withX(rowPose.getX() + (rowPose.getX() > 72 ? -UNSHORTCUT_LENGTH : UNSHORTCUT_LENGTH)),
                                                shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                () -> {if (trieurSubsystem.isEmpty()) this.cancel();}))),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem,true)
        );
    }
}
