package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class ShootAllAnyWay extends ConditionalCommand {
    public ShootAllAnyWay(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem) {
        super(
                new SequentialCommandGroup(
                        ShootAll.BeginShootAll(trieurSubsystem, chargeurSubsystem, shooterSubsystem, false),
                        new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem),
                        ShootAll.EndShootAll()),
                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false),
                trieurSubsystem::isEmpty);
    }
}
