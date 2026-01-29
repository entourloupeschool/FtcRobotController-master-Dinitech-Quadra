package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.InstantRumbleCustom;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * An autonomous command that prepares the moulin to shoot a predefined color pattern ("motif").
 * <p>
 * This command is designed for autonomous operation. It uses the color pattern from the
 * {@link VisionSubsystem} to calculate the optimal starting orientation for the moulin.
 * Unlike {@link ReadyMotif}, this command will throw an {@link IllegalArgumentException} if
 * no color pattern has been detected, as it is critical for autonomous sequences.
 * <p>
 * The logic positions the moulin based on where the <b>purple</b> artifact is in the sequence,
 * ensuring that when a sequential shooting command is run, the artifacts are fired in the
 * correct order.
 */
public class ReadyMotifAuto extends MoulinToPosition {
    private final VisionSubsystem visionSubsystem;

    /**
     * Creates a new ReadyMotifAuto command.
     *
     * @param trieurSubsystem The sorter subsystem for moulin control.
     * @param visionSubsystem The vision subsystem which must have a detected color motif.
     */
    public ReadyMotifAuto(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem){
        super(trieurSubsystem, 0, true); // Target is set dynamically, use shortest path.
        this.visionSubsystem = visionSubsystem;
    }

    /**
     * Calculates the target position based on the detected color motif and starts the rotation.
     */
    @Override
    public void initialize(){
        // Check if the color order has been detected by the vision system.
        if (!visionSubsystem.hasColorOrder()){
            moulinTargetPosition = 1;
        } else {
            // Get the detected color order.
            int colorsOrder = visionSubsystem.getColorsOrder();

            // Calculate the target position. The logic aligns the moulin based on the
            // location of the green artifact in the sequence.
            int greenPosition = trieurSubsystem.getClosestShootingPositionForColor(TrieurSubsystem.ArtifactColor.GREEN);

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
