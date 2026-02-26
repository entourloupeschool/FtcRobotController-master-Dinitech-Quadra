package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SLOW_DRIVE_SCALE;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

/**
 * A command for controlling the robot's drive base at a reduced speed.
 * <p>
 * This command is similar to {@link RobotCentricDrive} but calls the {@code teleSlowDrive}
 * method on the {@link DrivePedroSubsystem}. This provides a "slow mode" for more precise
 * maneuvering, using a fixed, lower power scale instead of the variable power from
 * the triggers.
 */
public class MaxPowerDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;

    /**
     * Creates a new TeleSlowDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public MaxPowerDrive(DrivePedroSubsystem drivePedroSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.setLastTeleDriverPowerScale(1);
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
