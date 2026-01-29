package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.ContinuousRumbleCustom;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.InstantRumbleCustom;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that manages the process of picking up an artifact and moving it away from the intake.
 * <p>
 * This command first executes the command to wait for and confirm the
 * presence of an artifact in the intake. Once the artifact has been successfully detected and
 * registered by the {@link TrieurSubsystem}, a {@link ConditionalCommand} is executed:
 * <ul>
 *     <li>If the artifact was newly registered, it runs {@link MoulinNextNext} to rotate the moulin
 *     two positions forward, moving the newly acquired artifact away from the intake and preparing
 *     an empty slot.</li>
 *     <li>If no new artifact was registered (e.g., the detection was a false positive or an old artifact),
 *     it triggers a {@link Rumble} to notify the driver.</li>
 * </ul>
 */
public class ArtefactPickAway extends SequentialCommandGroup {

    /**
     * Creates a new ArtefactPickAway command.
     *
     * @param trieurSubsystem  The sorter subsystem for artifact detection and moulin control.
     */
    public ArtefactPickAway(TrieurSubsystem trieurSubsystem, Command detectionCommand) {
        addCommands(
                // First, run the detection process
                detectionCommand,
                new InstantCommand(trieurSubsystem::registerArtefact),
                new MoulinNextNext(trieurSubsystem) // Command to run if a new artifact is present

//                // Then, conditionally rotate the moulin if a new artifact was registered
//                new ConditionalCommand(
//                        new MoulinNextNext(trieurSubsystem), // Command to run if a new artifact is present
//
//                        new InstantRumbleCustom(gamepadSubsystem, 3, 0.1),   // Command to run if no new artifact was registered
//                        trieurSubsystem::hasNewRegister      // The condition to check
//                )
        );
    }
}
