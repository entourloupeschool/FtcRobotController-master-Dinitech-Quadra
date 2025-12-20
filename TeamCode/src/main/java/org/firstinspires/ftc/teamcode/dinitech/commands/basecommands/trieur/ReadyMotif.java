package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.InstantRumbleCustom;
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
public class ReadyMotif extends MoulinToPosition {

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
            new InstantRumbleCustom(gamepadSubsystem, 3, 0.5).schedule();
            moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(trieurSubsystem.getClosestShootingPositionForColor(trieurSubsystem.getLastDetectedColor()), 1);
        } else {
            // Get the detected color order.
            String[] colorsOrder = visionSubsystem.getColorsOrder();

            // Find the position of the green artifact in the motif (1, 2, or 3).
            int greenPosition = -1;
            for (int i = 0; i < colorsOrder.length; i++){
                if (colorsOrder[i].equals("g")){
                    greenPosition = i + 1; // positions are 1-indexed
                    break;
                }
            }

            // Calculate the target position. The logic aligns the moulin based on the
            // location of the green artifact in the sequence.
            moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(trieurSubsystem.getClosestShootingPositionForColor(TrieurSubsystem.ArtifactColor.GREEN), 1);
            if (greenPosition == 2){
                // Adjust if green is the second artifact in the motif.
                moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(moulinTargetPosition, 3);
            } else if (greenPosition == 3){
                // Adjust if green is the third artifact in the motif.
                moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(moulinTargetPosition, 5);
            }
        }

        makeShort = false; // Always rotate forward.
        super.initialize(); // Execute the rotation.
    }
}
