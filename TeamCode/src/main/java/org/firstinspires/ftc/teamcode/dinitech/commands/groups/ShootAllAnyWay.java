package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinAlmostRevolution;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class ShootAllAnyWay extends ConditionalCommand {
    public ShootAllAnyWay(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem) {
        super(
                new SequentialCommandGroup(
                        ShootAll.BeginShootAll(trieurSubsystem, chargeurSubsystem),
                        new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem),
                        ShootAll.EndShootAll(trieurSubsystem)),
//                new SequentialCommandGroup(
//                        ShootAll.BeginShootAll(trieurSubsystem, chargeurSubsystem, shooterSubsystem, false),
//                        new MoulinAlmostRevolution(trieurSubsystem),
//                        ShootAll.EndShootAll(trieurSubsystem)),
                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false),
                trieurSubsystem::isEmpty);
    }

    public ShootAllAnyWay(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
        super(
                new SequentialCommandGroup(
                        ShootAll.BeginShootAll(trieurSubsystem, chargeurSubsystem),
                        new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem),
                        ShootAll.EndShootAll(trieurSubsystem)),
//                new SequentialCommandGroup(
//                        ShootAll.BeginShootAll(trieurSubsystem, chargeurSubsystem),
//                        new MoulinAlmostRevolution(trieurSubsystem),
//                        ShootAll.EndShootAll(trieurSubsystem)),
                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, drivePedroSubsystem, hubsSubsystem),
                trieurSubsystem::isEmpty);
    }
}
