package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command that prepares the moulin to shoot a predefined color pattern ("motif").
 * <p>
 * This command uses the color pattern detected by the {@link VisionSubsystem} to calculate
 * a specific starting orientation for the moulin. The goal is to align the artifacts
 * correctly so that a subsequent shooting command (like {@link org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootMotif}) can fire them
 * in the correct sequence.
 * <p>
 * If the vision subsystem has not detected a color pattern, it executes a fallback behavior:
 * it rumbles the gamepad and moves the moulin to a default position relative to the last
 * detected artifact.
 */
public class ReadyMotif extends MoulinToPositionLoose {

    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;


    /**
     * Creates a new ReadyMotif command.
     *
     * @param trieurSubsystem  The sorter subsystem for moulin control.
     * @param visionSubsystem  The vision subsystem to get the color motif from.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ReadyMotif(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem){
        super(trieurSubsystem, 0, false); // Target is set dynamically in initialize()
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    /**
     * Calculates the target position based on the detected color motif and starts the rotation.
     */
    @Override
    public void initialize(){
        // Check if the color order has been detected by the vision system.
        if (!visionSubsystem.hasColorOrder()){
            // Fallback: Rumble and move to a default position if no motif is detected.
            gamepadSubsystem.customRumble(new Gamepad.RumbleEffect.Builder()
                    .addStep(0.5, 0.5, 10)
                    .build(), 3);
            moulinTargetPosition = trieurSubsystem.getMoulinPosition();
        } else {
            // Get the detected color order.
            int colorsOrder = visionSubsystem.getColorsOrder();

            // Calculate the target position. The logic aligns the moulin based on the
            // location of the green artifact in the sequence.
            int greenPosition = trieurSubsystem.getPosWithColor(TrieurSubsystem.ArtifactColor.GREEN);

            if (greenPosition == -1){
                // Fallback if green position is not found.
                moulinTargetPosition = 1;
                makeShort = false; // Always rotate forward.
                super.initialize(); // Execute the rotation.
                return;
            }

            if (colorsOrder == 21){
                moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(greenPosition, 2);
            } else if (colorsOrder == 22){
                // Adjust if green is the second artifact in the motif.
                moulinTargetPosition = greenPosition;
            } else {
                // Adjust if green is the third artifact in the motif.
                moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(greenPosition, 2);
            }
        }

        makeShort = false; // Always rotate forward.
        super.initialize(); // Execute the rotation.
    }
}
