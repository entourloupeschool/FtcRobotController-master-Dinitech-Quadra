package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitCloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ReadyTrieurForPick extends ConditionalCommand {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     */
    public ReadyTrieurForPick(TrieurSubsystem trieurSubsystem) {
        super(
                // if condition is true.
                new WaitCloseTrappe(trieurSubsystem),

                // if condition is false.
                new SequentialCommandGroup(
                        new CloseTrappe(trieurSubsystem),
                        new MoulinNext(trieurSubsystem)),
                
                // Condition.
                () -> Moulin.isStoragePosition(trieurSubsystem.getMoulinPosition())
        );
    }
}
