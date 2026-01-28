package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that performs a full calibration sequence for the moulin.
 * <p>
 * This command ensures the moulin's physical state is perfectly synchronized with its
 * internal logical state. It is a critical routine to run at the beginning of a match
 * or after a potential desynchronization event.
 * <p>
 * The sequence is as follows:
 * <ol>
 *     <li><b>{@link MoulinCalibrate}:</b> Rotates the moulin until the magnetic home switch is triggered.
 *     This finds a known physical reference point.</li>
 *     <li><b>{@link MoulinNext}:</b> Rotates the moulin one position forward from the magnetic switch.
 *     This moves it to the first official storage slot.</li>
 *     <li><b>{@link InstantCommand}:</b> Executes {@code hardSetMoulinPosition(1)} on the subsystem.
 *     This forcefully sets the internal software position to 1, completing the synchronization.</li>
 * </ol>
 */
public class MoulinCalibrationSequence extends SequentialCommandGroup {

    /**
     * Creates a new MoulinCalibrationSequence command.
     *
     * @param trieurSubsystem The sorter subsystem to be calibrated.
     */
    public MoulinCalibrationSequence(TrieurSubsystem trieurSubsystem) {
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new CloseTrappe(trieurSubsystem),
                                new WaitCommand(200)),
                        new InstantCommand(),
                        trieurSubsystem::isTrappeOpen
                ),
                new MoulinCalibrate(trieurSubsystem),
                new InstantCommand(() -> trieurSubsystem.hardSetMoulinPosition(6), trieurSubsystem),
                new MoulinNext(trieurSubsystem)
        );
    }
}
