package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_POSITION_VERY_LOOSE_TOLERANCE;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinToPositionMargin;
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
public class ReadyMotif extends MoulinToPositionMargin {
    private final VisionSubsystem visionSubsystem;

    /**
     * Creates a new ReadyMotif command.
     *
     * @param trieurSubsystem  The sorter subsystem for moulin control.
     * @param visionSubsystem  The vision subsystem to get the color motif from.
     */
    public ReadyMotif(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem){
        super(trieurSubsystem, -1, false, MOULIN_POSITION_VERY_LOOSE_TOLERANCE); // Target is set dynamically in initialize()
        this.visionSubsystem = visionSubsystem;
    }

    /**
     * Calculates the target position based on the detected color motif and starts the rotation.
     */
    @Override
    public void initialize(){
        int motif = visionSubsystem.getCachedMotif();
        int greenPosition = trieurSubsystem.getPosWithColor(TrieurSubsystem.ArtifactColor.GREEN);

        if (motif == -1 || greenPosition == -1){
            // Fallback: Rumble and move to a default position if no motif is detected or no green artefacts stored.
            onCantMotif();
            super.initialize();
        } else {
            if (motif == 21){
                super.moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(greenPosition, 2);
            } else if (motif == 22){
                // Adjust if green is the second artifact in the motif.
                super.moulinTargetPosition = greenPosition;
            } else {
                // Adjust if green is the third artifact in the motif.
                super.moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(greenPosition, 2);
            }
            super.initialize(); // Execute the rotation.
        }
    }


    protected void onCantMotif() {
    }
}
