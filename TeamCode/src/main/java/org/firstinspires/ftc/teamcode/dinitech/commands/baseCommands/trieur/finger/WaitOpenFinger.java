package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Finger;

public class WaitOpenFinger extends ConditionalCommand {
    public WaitOpenFinger(TrieurSubsystem trieurSubsystem) {
        super(
                new InstantCommand(),
                new SequentialCommandGroup(
                        new OpenFinger(trieurSubsystem),
                        new WaitCommand(Finger.FINGER_OPEN_TIME)),
                trieurSubsystem::isFingerOpen);
    }
}
