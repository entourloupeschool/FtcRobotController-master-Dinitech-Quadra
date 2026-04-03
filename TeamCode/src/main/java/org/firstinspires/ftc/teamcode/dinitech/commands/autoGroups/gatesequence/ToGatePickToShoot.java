package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.WAIT_FOR_3BALL;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToGatePickToShoot extends SequentialCommandGroup {

    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, HubsSubsystem hubsSubsystem, Pose GatePickPose, Pose shootPose, boolean shortcutBackPath){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.curve(drivePedroSubsystem,
                                GatePickPose.withX(GatePickPose.getX() + (GatePickPose.getX() > 72 ? -1.7*TILE_DIM : 1.7*TILE_DIM)),
                                GatePickPose, 1, true)),

                new ParallelCommandGroup(
                        new RamassageAuto(trieurSubsystem, visionSubsystem, chargeurSubsystem, true),
                        new SequentialCommandGroup(
                                OptimalPath.line(drivePedroSubsystem,
                                        GatePickPose.withY(GatePickPose.getY() - 2).withHeading(GatePickPose.getHeading() / 2), 1, true),
                                new SetVelocityShooter(shooterSubsystem, ShooterSubsystem.linearSpeedFromPedroRange(shootPose.distanceFrom(hubsSubsystem.getTeam().getBasketPose()))),
                                new ParallelRaceGroup(
                                        new WaitCommand(WAIT_FOR_3BALL),
                                        new WaitUntilCommand(trieurSubsystem::isFull)),
                                // Go to Shooting Pos
                                shortcutBackPath ?
                                        OptimalPath.line(drivePedroSubsystem,
                                                shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                                        OptimalPath.curve(drivePedroSubsystem,
                                                        GatePickPose.withX(GatePickPose.getX() + (GatePickPose.getX() > 72 ? -2.1*TILE_DIM : 2.1*TILE_DIM)),
                                                        shootPose, 1, true)
                                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}))),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true)
        );
    }
}
