package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SLOW_DRIVE_SCALE;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * A command for controlling the robot's drive base at a reduced speed.
 * <p>
 * This command is similar to {@link RobotCentricDrive} but calls the {@code teleSlowDrive}
 * method on the {@link DrivePedroSubsystem}. This provides a "slow mode" for more precise
 * maneuvering, using a fixed, lower power scale instead of the variable power from
 * the triggers.
 */
public class SlowDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final GamepadWrapper driver;

    /**
     * Creates a new TeleSlowDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public SlowDrive(DrivePedroSubsystem drivePedroSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        driver = gamepadSubsystem.driver;

        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.SLOW);
    }

    /**
     * Executes the slow drive command. This method is called repeatedly by the command scheduler.
     */
    @Override
    public void execute(){
        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), SLOW_DRIVE_SCALE,drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);
    }
}
