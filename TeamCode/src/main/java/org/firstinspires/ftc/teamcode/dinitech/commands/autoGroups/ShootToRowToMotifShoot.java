package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_OPEN_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_AT_END_ROW;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ShootToRowToMotifShoot extends SequentialCommandGroup {

    public ShootToRowToMotifShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose InitPose, Pose RowPose, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, shooterVelocity),
                        new ReadyTrieurForPick(trieurSubsystem),
                        // go to first row of artefacts
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        RowPose)
                                ).setLinearHeadingInterpolation(InitPose.getHeading(), RowPose.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME/1.8).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem),
                                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem)
                        ),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? LENGTH_X_ROW : -LENGTH_X_ROW)))
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), RowPose.getHeading()).build(),
                                        MAX_POWER_ROW_PICK_ARTEFACTS, false),
                                new WaitCommand(WAIT_AT_END_ROW),
                                // Go to Shooting Pos
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                InitPose)
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), InitPose.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true)))
        );
    }

    public ShootToRowToMotifShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose InitPose, Pose RowPose, Pose endPose, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, shooterVelocity),
                        new ReadyTrieurForPick(trieurSubsystem),
                        // go to first row of artefacts
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        RowPose)
                                ).setLinearHeadingInterpolation(InitPose.getHeading(), RowPose.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME/1.8).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem),
                                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                                new OpenTrappe(trieurSubsystem),
                                new WaitCommand(TRAPPE_OPEN_TIME)
                        ),
                        new SequentialCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                RowPose.withX(RowPose.getX() + (RowPose.getX() > 72 ? LENGTH_X_ROW : -LENGTH_X_ROW)))
                                        ).setLinearHeadingInterpolation(RowPose.getHeading(), RowPose.getHeading()).build(),
                                        MAX_POWER_ROW_PICK_ARTEFACTS, false),
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
