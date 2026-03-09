package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

public class ResetHeadingFCDrive extends SetHeadingFCDrive {
    public ResetHeadingFCDrive(DrivePedroSubsystem drivePedroSubsystem) {
        super(drivePedroSubsystem, 0);
    }
}
