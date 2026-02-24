package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedRevolution extends SequentialCommandGroup {

    public MoulinHighSpeedRevolution(TrieurSubsystem trieurSubsystem) {
        addCommands(
            new MoulinNextShoot(trieurSubsystem),
            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
            new MoulinNextNext(trieurSubsystem),
            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
            new MoulinNextNext(trieurSubsystem));
    }
}
