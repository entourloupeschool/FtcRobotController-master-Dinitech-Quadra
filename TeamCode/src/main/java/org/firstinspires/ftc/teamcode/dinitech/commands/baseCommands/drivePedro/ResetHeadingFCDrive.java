package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;


/**
 * A command for reseting the heading of the robot.
 *
 */
public class ResetHeadingFCDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public ResetHeadingFCDrive(DrivePedroSubsystem drivePedroSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.setHeading(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
