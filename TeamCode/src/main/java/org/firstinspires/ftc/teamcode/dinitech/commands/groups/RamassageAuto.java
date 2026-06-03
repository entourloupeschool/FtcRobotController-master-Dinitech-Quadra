package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem.INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.WaitReadyShootTrappeFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class RamassageAuto extends SequentialCommandGroup {

    public RamassageAuto(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem,
                         GamepadSubsystem gamepadSubsystem, ChargeurSubsystem chargeurSubsystem, boolean shouldInversePowerChargeur) {
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new MaxPowerChargeur(chargeurSubsystem)),
                new TryDetectArtefactOptimizedRumble(trieurSubsystem, gamepadSubsystem),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimizedRumble(trieurSubsystem, gamepadSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimizedRumble(trieurSubsystem, gamepadSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ParallelCommandGroup(
                        new PrepShootTrieur(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                        new RemovePotentialArtefactChargeur(trieurSubsystem, chargeurSubsystem, shouldInversePowerChargeur))

        );
    }

    public RamassageAuto(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem,
                         ChargeurSubsystem chargeurSubsystem, boolean shouldInversePowerChargeur) {
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new MaxPowerChargeur(chargeurSubsystem)),
                new TryDetectArtefactOptimized(trieurSubsystem),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimized(trieurSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimized(trieurSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ParallelCommandGroup(
                        new PrepShootTrieur(trieurSubsystem, visionSubsystem),
                        new RemovePotentialArtefactChargeur(trieurSubsystem, chargeurSubsystem, shouldInversePowerChargeur))
        );
    }

    public RamassageAuto(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem, boolean shouldInversePowerChargeur) {
        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        new MaxPowerChargeur(chargeurSubsystem)),
                new TryDetectArtefactOptimized(trieurSubsystem),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimized(trieurSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextEmptyStorage(trieurSubsystem),
                                new TryDetectArtefactOptimized(trieurSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ParallelCommandGroup(
                        new WaitReadyShootTrappeFinger(trieurSubsystem),
                        new RemovePotentialArtefactChargeur(trieurSubsystem, chargeurSubsystem, shouldInversePowerChargeur))

        );
    }
}
