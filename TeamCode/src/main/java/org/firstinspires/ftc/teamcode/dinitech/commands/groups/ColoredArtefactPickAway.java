package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that picks up an artifact and moves it away, but only if it's a colored artifact.
 * <p>
 * This command is a more specific version of {@link ArtefactPickAway}. It first runs the
 * {@link DetectArtefact} process. Afterwards, it checks a more stringent condition:
 * <ul>
 *     <li>If a new <i>colored</i> (green or purple) artifact was registered, it executes
 *     {@link MoulinNextNext} to rotate the moulin, preparing the next slot.</li>
 *     <li>If no new colored artifact was registered (e.g., the artifact was of an unknown color
 *     or the detection failed), it triggers a {@link Rumble} to notify the driver.</li>
 * </ul>
 */
public class ColoredArtefactPickAway extends SequentialCommandGroup {

    /**
     * Creates a new ColoredArtefactPickAway command.
     *
     * @param trieurSubsystem  The sorter subsystem for artifact detection and moulin control.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ColoredArtefactPickAway(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        addCommands(
                // First, run the detection process
                new DetectArtefact(trieurSubsystem, gamepadSubsystem),
                // Then, conditionally rotate the moulin only if a new COLORED artifact was registered
                new ConditionalCommand(
                        new MoulinNextNext(trieurSubsystem), // Command to run if a new colored artifact is present
                        new Rumble(gamepadSubsystem, 3, 1),   // Command to run if no new colored artifact was registered
                        trieurSubsystem::hasNewColoredRegister // The more specific condition to check
                )
        );
    }
}
