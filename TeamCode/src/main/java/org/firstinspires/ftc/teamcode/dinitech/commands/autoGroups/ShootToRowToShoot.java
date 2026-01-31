package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_OPEN_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_AT_END_ROW;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootTimeAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ShootToRowToShoot extends SequentialCommandGroup {

    public ShootToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, GamepadSubsystem gamepadSubsystem, Pose InitPose, Pose RowPose, CommandBase commandBase, double shooterVelocity, double lengthBackup, double rowPower, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, shooterVelocity),
                        new ReadyTrieurForPick(trieurSubsystem),
                        // go to first row of artefacts
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        RowPose)
                                ).setLinearHeadingInterpolation(InitPose.getHeading(), RowPose.getHeading(), endTime).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_AUTO_TIMEOUT),
                                commandBase,
                                new OpenTrappe(trieurSubsystem),
                                new WaitCommand(TRAPPE_OPEN_TIME)),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? lengthBackup : -lengthBackup)))
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), RowPose.getHeading()).build(),
                                        rowPower, false),
                                new WaitCommand(WAIT_AT_END_ROW),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                InitPose)
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), InitPose.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true))),

                new ShootTimeAuto(trieurSubsystem, chargeurSubsystem)
        );
    }

    public ShootToRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, GamepadSubsystem gamepadSubsystem, Pose InitPose, Pose RowPose, Pose endPose, CommandBase commandBase, double shooterVelocity, double lengthBackup, double rowPower, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, shooterVelocity),
                        new ReadyTrieurForPick(trieurSubsystem),
                        // go to first row of artefacts
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        RowPose)
                                ).setLinearHeadingInterpolation(InitPose.getHeading(), RowPose.getHeading(), endTime).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_AUTO_TIMEOUT),
                                commandBase,
                                new OpenTrappe(trieurSubsystem),
                                new WaitCommand(TRAPPE_OPEN_TIME)
                        ),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? lengthBackup : -lengthBackup)))
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), RowPose.getHeading()).build(),
                                        rowPower, false),
                                new WaitCommand(WAIT_AT_END_ROW),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                endPose)
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), endPose.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)))
        );
    }
}
