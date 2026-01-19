package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SLOW_DRIVE_SCALE;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * A command for controlling the robot's drive base at a reduced speed.
 * <p>
 * This command is similar to {@link RobotCentricTeleDrive} but calls the {@code teleSlowDrive}
 * method on the {@link DriveSubsystem}. This provides a "slow mode" for more precise
 * maneuvering, using a fixed, lower power scale instead of the variable power from
 * the triggers.
 */
public class FieldCentricTeleSlowDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadWrapper driver;

    /**
     * Creates a new TeleSlowDrive command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public FieldCentricTeleSlowDrive(DriveSubsystem driveSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        driver = gamepadSubsystem.driver;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        driveSubsystem.setUsageState(DriveSubsystem.DriveUsageState.SLOW);
    }

    /**
     * Executes the slow drive command. This method is called repeatedly by the command scheduler.
     */
    @Override
    public void execute(){
        driveSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), SLOW_DRIVE_SCALE,true);
    }
}
