package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.PedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageTeleOp extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageTeleOp(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem,
                               GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ReadyTrieurForPick(trieurSubsystem),
                                new MaxPowerChargeur(chargeurSubsystem)),
                        new InstantCommand(()-> drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem)), drivePedroSubsystem),
                        new SequentialCommandGroup(
                                new InstantCommand(()-> shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem)), shooterSubsystem),
                                new StopShooter(shooterSubsystem))),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ConditionalCommand(
                        new MoulinNextNextVeryLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ConditionalCommand(
                        new MoulinNextNextVeryLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new ConditionalCommand(
                                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                                new InstantCommand(),
                                trieurSubsystem::wantsMotifShoot),
                        new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem)), shooterSubsystem)),
                new OpenWaitTrappe(trieurSubsystem),
                new ModeShootTeleOp(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem, hubsSubsystem));
    }
}
