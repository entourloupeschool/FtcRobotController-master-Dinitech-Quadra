package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRIEUR_TIME_BETWEEN_SHOTS;

/**
 * A conditional command that shoots artifacts in a specific sequence or "motif"
 * determined at the start of the match.
 * <p>
 * This command relies on the {@link VisionSubsystem} to have detected a color pattern from
 * the environment (e.g., from an AprilTag).
 * <ul>
 *     <li>If a color pattern has been detected, this command schedules a series of
 *     {@link ShootColor} commands, one for each color in the pattern, with a delay between each shot.</li>
 *     <li>If no pattern has been detected, it executes a {@link Rumble} command to signal
 *     that the action cannot be performed.</li>
 * </ul>
 */
public class ShootMotif extends ConditionalCommand {
    /**
     * Creates a new ShootMotif command.
     *
     * @param trieurSubsystem  The sorter subsystem, used by the scheduled ShootColor commands.
     * @param shooterSubsystem The shooter subsystem, used by the scheduled ShootColor commands.
     * @param visionSubsystem  The vision subsystem, which provides the color pattern.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ShootMotif(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                // Command to run if the color order has been detected
                new InstantCommand(() -> {
                    String[] colorsOrder = visionSubsystem.getColorsOrder();

                    // For each color in the detected order, schedule a ShootColor command
                    // followed by a wait period.
                    // Note: These commands are scheduled independently and will run sequentially
                    // due to the WaitCommand and subsystem requirements.
                    long delay = 0;
                    for (String color : colorsOrder) {
                        if (color.equals("g")) {
                            new WaitCommand(delay).andThen(new ShootColor(trieurSubsystem, shooterSubsystem, gamepadSubsystem, TrieurSubsystem.ArtifactColor.GREEN)).schedule();
                        } else if (color.equals("p")) {
                            new WaitCommand(delay).andThen(new ShootColor(trieurSubsystem, shooterSubsystem, gamepadSubsystem, TrieurSubsystem.ArtifactColor.PURPLE)).schedule();
                        }
                        delay += TRIEUR_TIME_BETWEEN_SHOTS;
                    }
                }),
                // Command to run if no color order is detected
                new Rumble(gamepadSubsystem, 3, 1),
                // Condition: Check if the vision subsystem has a color order
                visionSubsystem::hasColorOrder
        );
    }
}
