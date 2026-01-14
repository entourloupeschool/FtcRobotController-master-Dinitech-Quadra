package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A conditional command that attempts to shoot an artifact of a specific color.
 * <p>
 * This command checks if an artifact of the specified color is available in the
 * {@link TrieurSubsystem}.
 * <ul>
 *     <li>If a matching artifact is found, it executes a {@link ShootMoulinState} command to
 *     rotate the moulin to the appropriate shooting position and launch the artifact.</li>
 *     <li>If no matching artifact is found, it triggers a {@link Rumble} command to provide
 *     haptic feedback to the driver.</li>
 * </ul>
 */
public class ShootColor extends ConditionalCommand {

    /**
     * Creates a new ShootColor command.
     *
     * @param trieurSubsystem  The sorter subsystem, used to find artifacts by color.
     * @param shooterSubsystem The shooter subsystem, used to launch the artifacts.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     * @param inputColor       The {@link TrieurSubsystem.ArtifactColor} to find and shoot.
     */
    public ShootColor(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem,
            GamepadSubsystem gamepadSubsystem, TrieurSubsystem.ArtifactColor inputColor) {
        super(
                // Command to execute if a matching artifact is found
                new DropMoulinState(trieurSubsystem, shooterSubsystem,
                        trieurSubsystem.getClosestShootingPositionForColor(inputColor), true),
                // Command to execute if no matching artifact is found
                new Rumble(gamepadSubsystem, 3, 1),
                // Condition: Check if a valid shooting position exists for the specified color
                () -> trieurSubsystem.getClosestShootingPositionForColor(inputColor) > 0);
    }
}
