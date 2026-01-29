package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

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
public class TryDetectArtefact extends CommandBase {

    private final TrieurSubsystem trieurSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    private Gamepad.RumbleEffect waitRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.3, 0.3, 10)
            .build();
    private Gamepad.RumbleEffect unfoundRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.9, 0.9, 20)
            .build();;

    private int timeout;

    /**
     * Creates a new ShootColor command.
     *
     * @param trieurSubsystem  The sorter subsystem, used to find artifacts by color.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public TryDetectArtefact(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem, int timeout) {
        this.trieurSubsystem = trieurSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        trieurSubsystem.clearSamplesColorSensors();
    }

    @Override
    public void execute() {
        gamepadSubsystem.customRumble(waitRumbleEffect, 2);
        trieurSubsystem.updateColorSensors();
        timeout -= 1;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            if (timeout <= 0) {
                gamepadSubsystem.customRumble(unfoundRumbleEffect, 2);
            } else {
                trieurSubsystem.registerArtefact();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return trieurSubsystem.isArtefactInTrieur() || timeout <= 0;
    }
}
