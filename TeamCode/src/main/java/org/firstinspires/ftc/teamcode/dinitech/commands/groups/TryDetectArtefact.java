package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that attempts to shoot an artifact of a specific color.
 * <p>
 * This command checks if an artifact of the specified color is available in the
 * {@link TrieurSubsystem}.
 * <ul>
 *     <li>If a matching artifact is found, it executes a command to
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
    private Gamepad.RumbleEffect waitRumbleEffect;
    private Gamepad.RumbleEffect unfoundRumbleEffect;
    private int timeout;
    private boolean isFound;


    /**
     * Creates a new ShootColor command.
     *
     * @param trieurSubsystem  The sorter subsystem, used to find artifacts by color.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public TryDetectArtefact(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    @Override
    public void initialize() {
        trieurSubsystem.setNewRegister(false);
        trieurSubsystem.setNewColoredRegister(false);

        timeout = trieurSubsystem.getDetectTimeout();
        isFound = false;

        trieurSubsystem.clearSamplesColorSensors();

        waitRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 20)
                .build();
        unfoundRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.9, 0.9, 50)
                .addStep(0.5, 0.5, 20)
                .addStep(0.9, 0.9, 50)
                .addStep(0.5, 0.5, 20)
                .addStep(0.9, 0.9, 50)
                .build();;
    }

    @Override
    public void execute() {
        gamepadSubsystem.customRumble(waitRumbleEffect, 2, true);
        trieurSubsystem.updateColorSensors();

        if (trieurSubsystem.isArtefactInTrieur() && !isFound) {
            trieurSubsystem.registerArtefact();
            isFound = true;
        } else if (timeout == 0) {
            gamepadSubsystem.customRumble(unfoundRumbleEffect, 2, true);
        }

        timeout -= 1;
    }


    @Override
    public boolean isFinished() {
        return isFound || timeout <= 0;
    }
}
