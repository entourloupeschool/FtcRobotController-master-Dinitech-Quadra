package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A conditional command that attempts to shoot any available artifact.
 * <p>
 * This command first checks if there is any artifact of any color available in the
 * {@link TrieurSubsystem}.
 * <ul>
 *     <li>If an artifact is found, it executes a {@link ShootMoulinState} command to rotate the
 *     moulin to the closest shooting position and shoot the artifact.</li>
 *     <li>If no artifact is found, it executes a {@link Rumble} command to provide haptic
 *     feedback to the driver, indicating that no shot can be taken.</li>
 * </ul>
 */
public class ShootAny extends ConditionalCommand {

    /**
     * Creates a new ShootAny command.
     *
     * @param trieurSubsystem  The sorter subsystem used to find artifacts.
     * @param shooterSubsystem The shooter subsystem used to launch artifacts.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ShootAny(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                // Command to execute when condition is true (artifact found)
                new DropMoulinState(trieurSubsystem, shooterSubsystem, trieurSubsystem.getClosestShootingPositionAnyColor(), true),
                // Command to execute when condition is false (no artifact found)
                new Rumble(gamepadSubsystem, 3, 1),
                // Condition to evaluate: checks if there's any available shooting position
                () -> trieurSubsystem.getClosestShootingPositionAnyColor() > 0
        );

    }
}
