package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.RADIUS_RAMP_PICK;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem.INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MODE_RAMASSAGE_AUTO_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.WAIT_FOR_3BALL;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefactOptimized;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToGatePickToShoot extends SequentialCommandGroup {

    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose openRampPose, Pose gatePickPose, Pose shootPose, double timeAtGate, double shooterVelocity, boolean shortcutBackPath){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.curve(drivePedroSubsystem,
                                openRampPose.withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -2*TILE_DIM : 2*TILE_DIM)),
                                openRampPose, 1, true)),
                new ParallelCommandGroup(
                        new WaitCommand((long) timeAtGate),
                        new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new TrieurReadyEmptyStorage(trieurSubsystem),
                                        new MaxPowerChargeur(chargeurSubsystem)),
                                new TryDetectArtefactOptimized(trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 2), trieurSubsystem),
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem),
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister)),

                        new SequentialCommandGroup(
                                OptimalPath.curve(drivePedroSubsystem,
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -RADIUS_RAMP_PICK : RADIUS_RAMP_PICK)),
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? RADIUS_RAMP_PICK/2 : -RADIUS_RAMP_PICK/2)).withY(gatePickPose.getY() -RADIUS_RAMP_PICK*1.2),
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? RADIUS_RAMP_PICK/2 : -RADIUS_RAMP_PICK/2)).withHeading(gatePickPose.getHeading()/2),
                                        GATEPICK_POWER, true),
//                                OptimalPath.curve(drivePedroSubsystem,
//                                        gatePickPose.withY(gatePickPose.getY() -RADIUS_RAMP_PICK*1.1),
//                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -RADIUS_RAMP_PICK : RADIUS_RAMP_PICK)),
//                                        openRampPose,
//                                        GATEPICK_POWER, true),
                                OptimalPath.curve(drivePedroSubsystem,
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -RADIUS_RAMP_PICK : RADIUS_RAMP_PICK)),
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -RADIUS_RAMP_PICK : RADIUS_RAMP_PICK)).withY(gatePickPose.getY() -RADIUS_RAMP_PICK*1.2),
                                        GATEPICK_POWER, true).withParametricCallback(0.7,
                                        () -> {if (trieurSubsystem.isFull()) this.cancel();}),
                                OptimalPath.curve(drivePedroSubsystem,
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? RADIUS_RAMP_PICK/2 : -RADIUS_RAMP_PICK/2)).withY(gatePickPose.getY() -RADIUS_RAMP_PICK*1.2),
                                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? RADIUS_RAMP_PICK/2 : -RADIUS_RAMP_PICK/2)).withHeading(gatePickPose.getHeading()/2),
                                        GATEPICK_POWER, true).withParametricCallback(0.1,
                                        () -> {if (trieurSubsystem.isFull()) this.cancel();})
                                        .withParametricCallback(0.5,
                                        () -> {if (trieurSubsystem.isFull()) this.cancel();}),

                                new ParallelRaceGroup(
                                        new WaitCommand(WAIT_FOR_3BALL),
                                        new WaitUntilCommand(trieurSubsystem::isFull)))),
                new ParallelCommandGroup(
                        new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new ReadyMotif(trieurSubsystem, visionSubsystem),
                                        new InstantCommand(),
                                        ()->trieurSubsystem.wantsMotifShoot()),
                                new WaitOpenTrappe(trieurSubsystem)),

                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InverseMaxPowerChargeur(chargeurSubsystem),
                                                new WaitCommand(INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO)),
                                        new InstantCommand(),
                                        ()->trieurSubsystem.isFull()),
                                new StopChargeur(chargeurSubsystem))),
                // Go to Shooting Pos
                shortcutBackPath ?
                        OptimalPath.line(drivePedroSubsystem,
                                shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                        OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose.withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -2.1*TILE_DIM : 2.1*TILE_DIM)),
                                        shootPose, 1, true)
                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false)
        );
    }

    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose openRampPose, Pose endRampPose, Pose shootPose, double shooterVelocity, boolean shortcutBackPath){
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.curve(drivePedroSubsystem,
                                openRampPose.withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -2*TILE_DIM : 2*TILE_DIM)),
                                openRampPose, 1, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem),
                                        new TrieurReadyEmptyStorage(trieurSubsystem),
                                        new MaxPowerChargeur(chargeurSubsystem)),
                                new TryDetectArtefactOptimized(trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 2), trieurSubsystem),
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem),
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister)),

                        new SequentialCommandGroup(
                                OptimalPath.curve(drivePedroSubsystem,
                                        endRampPose
                                                .withX(endRampPose.getX() + (endRampPose.getX() > 72 ? RADIUS_RAMP_PICK/3 : -RADIUS_RAMP_PICK/3))
                                                .withY((endRampPose.getY() + openRampPose.getY())/2),
                                        endRampPose,
                                        GATEPICK_POWER, true),

                                new ParallelRaceGroup(
                                        new WaitCommand(WAIT_FOR_3BALL*2L),
                                        new WaitUntilCommand(trieurSubsystem::isFull)))),
                new ParallelCommandGroup(
                        new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new ReadyMotif(trieurSubsystem, visionSubsystem),
                                        new InstantCommand(),
                                        ()->trieurSubsystem.wantsMotifShoot()),
                                new WaitOpenTrappe(trieurSubsystem)),

                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InverseMaxPowerChargeur(chargeurSubsystem),
                                                new WaitCommand(INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO)),
                                        new InstantCommand(),
                                        ()->trieurSubsystem.isFull()),
                                new StopChargeur(chargeurSubsystem))),
                // Go to Shooting Pos
                shortcutBackPath ?
                        OptimalPath.line(drivePedroSubsystem,
                                shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                        OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose.withX(openRampPose.getX() + (openRampPose.getX() > 72 ? -2.1*TILE_DIM : 2.1*TILE_DIM)),
                                        shootPose, 1, true)
                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false)
        );
    }
}
