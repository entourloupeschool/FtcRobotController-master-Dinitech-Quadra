package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Finger;

public class WaitCloseFinger extends ConditionalCommand {
    public WaitCloseFinger(TrieurSubsystem trieurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new CloseFinger(trieurSubsystem),
                        new WaitCommand(Finger.FINGER_CLOSE_TIME)),
                new InstantCommand(),
                trieurSubsystem::isFingerOpen);
    }
}
