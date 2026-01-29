package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that attempts to shoot an artifact of a specific color.
 * <p>
 * This command checks if an artifact of the specified color is available in the
 * {@link TrieurSubsystem}.
 * <ul>
 *     <li>If a matching artifact is found, it executes a {@link DropMoulinState} command to
 *     rotate the moulin to the appropriate shooting position and launch the artifact.</li>
 *     <li>If no matching artifact is found, it triggers a {@link Rumble} command to provide
 *     haptic feedback to the driver.</li>
 * </ul>
 * <p>
 * The shooting position is evaluated once at initialization time to avoid race conditions.
 */
public class ShootColor extends CommandBase {

    private final TrieurSubsystem trieurSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    private final TrieurSubsystem.ArtifactColor inputColor;

    private CommandBase selectedCommand;

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
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        this.inputColor = inputColor;
    }

    @Override
    public void initialize() {
        // Evaluate position once at initialization
        int shootingPosition = trieurSubsystem.getClosestShootingPositionForColor(inputColor);

        if (shootingPosition > 0) {
            selectedCommand = new DropMoulinState(trieurSubsystem, shooterSubsystem, shootingPosition, true);
        } else {
            selectedCommand = new Rumble(gamepadSubsystem, 2, 1);
        }

        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }
}
