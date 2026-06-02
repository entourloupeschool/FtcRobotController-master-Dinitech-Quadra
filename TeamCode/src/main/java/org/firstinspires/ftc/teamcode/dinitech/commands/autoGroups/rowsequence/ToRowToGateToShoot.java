package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence;


import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.UNSHORTCUT_LENGTH;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToRowToGateToShoot extends SequentialCommandGroup {
    public ToRowToGateToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose rowPose, Pose shootPose, Pose openRampPose, double shooterVelocity, double lengthBackup, double rowPower, boolean shortcutBackPath, long timeAtGate){
        double backupForGatePush = openRampPose.getX() + TILE_DIM / 1.55*(openRampPose.getX() > 72 ? -1 : 1);
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.line(drivePedroSubsystem,
                                rowPose, 1, true)),

                new ParallelCommandGroup(
                        new RamassageAuto(trieurSubsystem, visionSubsystem, chargeurSubsystem, false),
                        new SequentialCommandGroup(
                                OptimalPath.line(drivePedroSubsystem,
                                        rowPose.withX(rowPose.getX() + lengthBackup*(rowPose.getX() > 72 ? 1 : -1)), rowPower, true),
                                OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose
                                                .withX(backupForGatePush)
                                                .withY(openRampPose.getY() - 2.5),
                                        openRampPose
                                                .withX(backupForGatePush)
                                                .withY(openRampPose.getY() + 2.5),
                                        openRampPose, 1, true),

                                new WaitCommand(timeAtGate),
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                shortcutBackPath ?
                                        OptimalPath.line(drivePedroSubsystem,
                                                shootPose, 1, true)
                                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                                        OptimalPath.curve(drivePedroSubsystem,
                                                rowPose.withX(rowPose.getX() + UNSHORTCUT_LENGTH*(rowPose.getX() > 72 ? -1 : 1)),
                                                shootPose, 1, true)
                                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}))),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem,true, false, false)
        );
    }
}
