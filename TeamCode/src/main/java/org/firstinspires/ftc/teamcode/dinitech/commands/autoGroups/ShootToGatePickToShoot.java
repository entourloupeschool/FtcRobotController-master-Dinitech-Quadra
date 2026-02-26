package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_AT_END_ROW;

import com.arcrobotics.ftclib.command.CommandBase;
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
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooterPower;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAlmostRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ShootToGatePickToShoot extends SequentialCommandGroup {
    public ShootToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose GatePickPose, Pose GatePickEndPose, Pose endPose, CommandBase commandBase, double shooterVelocity, double gatePower, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new StopShooterPower(shooterSubsystem),
                        new ReadyTrieurForPick(trieurSubsystem),
                        // go to first row of artefacts
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
                        new SequentialCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new ModeRamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, MODE_RAMASSAGE_AUTO_TIMEOUT),
                                commandBase),
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        GatePickEndPose))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        GatePickEndPose.getHeading(),
                                        LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                gatePower, false)),

                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, shooterVelocity),
                        new OpenWaitTrappe(trieurSubsystem),
                        new SequentialCommandGroup(
                                new WaitCommand(WAIT_AT_END_ROW),
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

                new ShootHighSpeedRevolution(trieurSubsystem)
        );
    }
}
