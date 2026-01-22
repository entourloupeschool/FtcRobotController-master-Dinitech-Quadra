package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * A command for controlling the robot's drive base during tele-operated period.
 * <p>
 * This command reads the joystick and trigger values from the driver's gamepad and
 * uses them to control the robot's movement. It translates the left stick for strafing
 * and forward/backward motion, the right stick for rotation, and the right trigger for
 * scaling the overall power.
 */
public class RobotCentricDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadWrapper driver;

    /**
     * Creates a new TeleDrive command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public RobotCentricDrive(DriveSubsystem driveSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.driver = gamepadSubsystem.driver;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        driveSubsystem.setDriveUsage(DriveSubsystem.DriveUsage.TELE);
        driveSubsystem.setDriveReference(DriveSubsystem.DriveReference.ROBOT);
    }

    /**
     * Executes the drive command. This method is called repeatedly by the command scheduler.
     */
    @Override
    public void execute(){
        driveSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), driver.getRightTriggerValue(), driveSubsystem.getDriveReference() == DriveSubsystem.DriveReference.FC);
    }
}
