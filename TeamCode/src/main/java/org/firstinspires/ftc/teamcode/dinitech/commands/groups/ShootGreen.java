package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A specialized command for shooting a green artifact.
 * <p>
 * This class extends {@link ShootColor} and is pre-configured to specifically target
 * and shoot artifacts identified as {@link TrieurSubsystem.ArtifactColor#GREEN}.
 * If no green artifact is available, it provides haptic feedback to the driver.
 */
public class ShootGreen extends ShootColor {

    /**
     * Creates a new ShootGreen command.
     *
     * @param trieurSubsystem  The sorter subsystem for finding the green artifact.
     * @param shooterSubsystem The shooter subsystem for launching the artifact.
     * @param gamepadSubsystem The gamepad subsystem for haptic feedback.
     */
    public ShootGreen(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem,
                      GamepadSubsystem gamepadSubsystem) {
        super(trieurSubsystem, shooterSubsystem, gamepadSubsystem, TrieurSubsystem.ArtifactColor.GREEN);
    }
}
