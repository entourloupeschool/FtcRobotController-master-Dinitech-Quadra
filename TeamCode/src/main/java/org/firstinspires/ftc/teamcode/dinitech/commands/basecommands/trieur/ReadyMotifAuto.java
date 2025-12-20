package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

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
     *
     * @throws IllegalArgumentException if no color motif has been detected by the vision subsystem.
     */
    @Override
    public void initialize(){
        if (!visionSubsystem.hasColorOrder()){
            throw new IllegalArgumentException("Cannot run ReadyMotifAuto: No color motif detected.");
        }

        String[] colorsOrder = visionSubsystem.getColorsOrder();

        // Find the 1-indexed position of the purple artifact in the motif.
        int purplePosition = -1;
        for (int i = 0; i < colorsOrder.length; i++){
            if (colorsOrder[i].equals("p")){
                purplePosition = i + 1;
                break;
            }
        }

        // Determine the starting position of the moulin so that a sequence of
        // "shoot, rotate 2" actions will fire the artifacts in the correct order.
        moulinTargetPosition = trieurSubsystem.getClosestShootingPositionForColor(TrieurSubsystem.ArtifactColor.PURPLE);
        if (purplePosition == 2) {
            // If purple is second, the moulin needs to be 2 steps before the purple shooting position.
            moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(moulinTargetPosition, 2);
        } else if (purplePosition == 3) {
            // If purple is third, the moulin needs to be 4 steps before the purple shooting position.
            moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(moulinTargetPosition, 4);
        }

        makeShort = true; // Use the shortest path for efficiency in auto.

        super.initialize();
    }
}
