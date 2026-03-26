package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class ShootAllAnyWay extends ConditionalCommand {
    public ShootAllAnyWay(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        super(
                new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem)),
                new ShootAll(trieurSubsystem, shooterSubsystem),
                trieurSubsystem::isEmpty);
    }
}
