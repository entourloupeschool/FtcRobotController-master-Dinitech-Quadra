package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;


/**
 * A command for reseting the heading of the robot.
 *
 */
public class IncrementHeadingFCDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final double newHeading;

    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public IncrementHeadingFCDrive(DrivePedroSubsystem drivePedroSubsystem, double newHeading) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.newHeading = newHeading;
        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.setHeading(drivePedroSubsystem.getHeading() + newHeading);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
