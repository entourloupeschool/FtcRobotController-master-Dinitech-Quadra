package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

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
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadWrapper driver;

    private Localizer orinalLocalizer;

    /**
     * Creates a new TeleDriveHybrid command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public FieldCentricDrive(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driver = gamepadSubsystem.driver;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        driveSubsystem.setDriveUsage(DriveSubsystem.DriveUsage.TELE);
        driveSubsystem.setDriveReference(DriveSubsystem.DriveReference.FC);


//        orinalLocalizer = driveSubsystem.getLocalizer();
//        driveSubsystem.setLocalizer(new AprilTagLocalizer(orinalLocalizer, visionSubsystem));
    }

    /**
     * Executes the drive command. This method is called repeatedly by the command scheduler.
     */
    @Override
    public void execute(){
        // Update localizer every cycle to get current heading
        driveSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), driver.getRightTriggerValue(), driveSubsystem.getDriveReference() == DriveSubsystem.DriveReference.FC);
    }

    @Override
    public void end(boolean interrupted){
//        driveSubsystem.setLocalizer(orinalLocalizer);
    }

}
