package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.REVOLUTION_MOULIN_TICKS;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;


public class MoulinRevolution extends MoulinToIncrementMargin {

    /**
     * Creates a new MoulinRevolution command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinRevolution(TrieurSubsystem trieurSubsystem) {
        super(trieurSubsystem, REVOLUTION_MOULIN_TICKS);
    }
}
