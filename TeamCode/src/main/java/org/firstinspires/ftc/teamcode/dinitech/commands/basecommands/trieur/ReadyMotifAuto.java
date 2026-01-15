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
            greenPosition = trieurSubsystem.getClosestShootingPositionForColor(TrieurSubsystem.ArtifactColor.GREEN);

            if (greenPosition == -1){
                // Fallback if green position is not found.
                moulinTargetPosition = 1;
                makeShort = false; // Always rotate forward.
                super.initialize(); // Execute the rotation.       
                return;
            }
            
            moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(greenPosition, 1);
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
