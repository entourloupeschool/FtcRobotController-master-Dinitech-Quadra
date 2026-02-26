package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.function.Supplier;

/**
 * An instant command that toggles the drive mode between normal tele-op and vision-assisted "locked" driving.
 * <p>
 * When executed, this command checks the AprilTag lock state of the {@link DrivePedroSubsystem}.
 * <ul>
 *     <li>If the drive is in vision-locked mode, it restores the default command to {@link RobotCentricDrive}
 *     for normal, robot-centric control.</li>
 *     <li>If the drive is in normal mode, it sets the default command to {@link VisionAimLockedDrive},
 *     enabling the vision-assisted auto-aim behavior, and triggers a rumble for haptic feedback.</li>
 * </ul>
 * The command finishes immediately after setting the new default command.
 */
public class SwitchVisionDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    private final Supplier<Pose> goalPoseSupplier;

    /**
     * Creates a new ToggleVisionDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     * @param visionSubsystem  The vision subsystem required by the locked drive mode.
     * @param gamepadSubsystem The gamepad subsystem for providing feedback.
     * @param goalPoseSupplier Supplier for the goal pose
     */
    public SwitchVisionDrive(DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem,
                             GamepadSubsystem gamepadSubsystem, Supplier<Pose> goalPoseSupplier) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        this.goalPoseSupplier = goalPoseSupplier;
    }

    /**
     * Executes the toggle logic. This is run once when the command is scheduled.
     */
    @Override
    public void initialize() {
        if (drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.PEDRO_AIM){
            // If vision is locked, switch back to normal drive.
            drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem));
        } else if (drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.NONE){
            // If not locked, switch to vision-locked drive and provide feedback.
            drivePedroSubsystem.setDefaultCommand(new VisionAimLockedDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));
        } else if (drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.VISION_AIM) {
            drivePedroSubsystem.setDefaultCommand(new PedroAimLockedDrive(drivePedroSubsystem, gamepadSubsystem, goalPoseSupplier));
        }
    }

    /**
     * This command has no duration and is finished immediately after execution.
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
