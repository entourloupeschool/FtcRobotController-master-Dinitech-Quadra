package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.END_WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.WaitReadyShootTrappeFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class ShootAllAnyWay extends ConditionalCommand {
    public ShootAllAnyWay(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new WaitReadyShootTrappeFinger(trieurSubsystem),
                        new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem),
                        new WaitCommand(END_WAIT_HIGH_SPEED_TRIEUR)),
                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false),
                trieurSubsystem::isEmpty);
    }
}
