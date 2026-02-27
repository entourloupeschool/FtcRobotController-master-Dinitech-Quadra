package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.function.BooleanSupplier;

/**
 * A field-centric tele-operated drive command.
 * <p>
 * This command provides a more intuitive driving experience by making the robot move
 * relative to the field's coordinate system, rather than its own orientation. Pushing the
 * left stick forward will always move the robot away from the driver, regardless of which
 * way the robot is facing.
 * <p>
 * How it works:
 * <ol>
 *     <li>At initialization, it captures the robot's starting heading.</li>
 *     <li>In the {@code execute} loop, it reads the joystick inputs, which are considered
 *     "field-centric" (e.g., Y is forward on the field, X is right on the field).</li>
 *     <li>It calculates the robot's current heading relative to its starting heading.</li>
 *     <li>It applies a rotation transformation to the joystick input vector to convert the desired
 *     field-centric movement into the robot's local coordinate frame.</li>
 *     <li>The resulting robot-centric velocities are sent to the drive subsystem.</li>
 * </ol>
 * The right stick still controls rotation as in a normal robot-centric drive.
 */
public class FieldCentricDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final GamepadWrapper driver;

    /**
     * Creates a new TeleDriveHybrid command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public FieldCentricDrive(DrivePedroSubsystem drivePedroSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.driver = gamepadSubsystem.getDriver();

        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.TELE);
        drivePedroSubsystem.setDriveReference(DrivePedroSubsystem.DriveReference.FC);
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.NONE);

        drivePedroSubsystem.setLastTeleDriverPowerScale(1);
    }

    /**
     * Executes the drive command. This method is called repeatedly by the command scheduler.
     */
    @Override
    public void execute(){
        // Update localizer every cycle to get current heading
        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), driver.getRightTriggerValue(), drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);
    }
}
