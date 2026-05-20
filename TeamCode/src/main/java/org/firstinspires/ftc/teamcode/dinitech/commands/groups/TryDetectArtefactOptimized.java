package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that attempts to detect an artifact.
 * <p>
 * This command checks if an artifact is available in the {@link TrieurSubsystem}.
 * <ul>
 *     <li>If an artifact is found, it registers it in the sorter.</li>
 *     <li>If the command is still waiting or reaches timeout, subclasses can react by overriding
 *     the hook methods.</li>
 * </ul>
 * 
 * <b>Optimization:</b> Checks artifact presence before updating color sensors,
 * avoiding expensive color processing when no artifact is nearby.
 */
public class TryDetectArtefactOptimized extends CommandBase {

    private final TrieurSubsystem trieurSubsystem;
    private int timeout;
    private boolean isFound;


    /**
     * Creates a new detection command.
     *
     * @param trieurSubsystem The sorter subsystem, used to detect and register artifacts.
     */
    public TryDetectArtefactOptimized(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    @Override
    public void initialize() {
        trieurSubsystem.setNewRegister(false);
        trieurSubsystem.setNewColoredRegister(false);
        trieurSubsystem.clearSamplesColorSensors();

        timeout = trieurSubsystem.getDetectionTimeout();
        isFound = false;
    }

    @Override
    public void execute() {
        // Check for artifact presence FIRST (cheap distance/sensor operation)
        if (trieurSubsystem.isArtefactInTrieur() && !isFound) {
            // Only update color sensors when artifact is detected (expensive operation)
            if (trieurSubsystem.wantsMotifShoot()) trieurSubsystem.updateColorSensors();
            trieurSubsystem.registerArtefact();
            isFound = true;
        } else if (timeout == 0) {
            onTimeoutReached();
        } else {
            onWaitingForArtefact();
        }

        timeout -= 1;
    }

    protected void onTimeoutReached() {
    }

    protected void onWaitingForArtefact() {
    }


    @Override
    public boolean isFinished() {
        return isFound || timeout <= 0;
    }
}
