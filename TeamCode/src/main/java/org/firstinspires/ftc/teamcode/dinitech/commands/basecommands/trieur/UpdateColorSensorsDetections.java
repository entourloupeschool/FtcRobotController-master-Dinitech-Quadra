package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that collects a specified number of color sensor samples to ensure a reliable
 * reading before identifying an artifact.
 * <p>
 * This command is designed to deal with sensor noise by averaging multiple readings.
 * It works as follows:
 * <ol>
 *     <li>On initialization, it clears any old sensor data from the subsystem.</li>
 *     <li>In a loop, it continuously asks the subsystem to read new values from the color sensors.</li>
 *     <li>The command finishes once a target number of samples has been collected.</li>
 *     <li>Upon finishing, it calls the subsystem's {@code registerArtefact()} method to process the
 *     collected data and officially determine and store the artifact's color.</li>
 * </ol>
 */
public class UpdateColorSensorsDetections extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final int numberSample;

    /**
     * Creates a new UpdateColorSensorsDetections command.
     *
     * @param trieurSubsystem The sorter subsystem to interact with.
     * @param numberSample    The target number of samples to collect before finishing.
     */
    public UpdateColorSensorsDetections(TrieurSubsystem trieurSubsystem, int numberSample){
        this.trieurSubsystem = trieurSubsystem;
        this.numberSample = numberSample;
        addRequirements(trieurSubsystem);
    }

    /**
     * Clears previous color sensor data at the start of the command.
     */
    @Override
    public void initialize(){
        trieurSubsystem.clearSamplesColorSensors();
    }

    /**
     * Collects a new color sensor sample in each execution loop.
     */
    @Override
    public void execute(){
        trieurSubsystem.updateColorSensors();
    }

    /**
     * The command is finished once the required number of samples has been collected.
     *
     * @return True if enough samples have been gathered.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.getColorSensorSampleCount() >= numberSample;
    }

    /**
     * Triggers the final artifact registration process after collecting samples.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // If not interrupted, process the collected data to register the artifact.
        if (!interrupted) {
            trieurSubsystem.registerArtefact();
        }
    }
}
