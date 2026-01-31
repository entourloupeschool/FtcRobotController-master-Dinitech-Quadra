package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

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
public class ContinuousUpdateColorSensorsDetectionsInAuto extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private int timeout;


    /**
     * Creates a new UpdateColorSensorsDetections command.
     *
     * @param trieurSubsystem The sorter subsystem to interact with.
     */
    public ContinuousUpdateColorSensorsDetectionsInAuto(TrieurSubsystem trieurSubsystem, int timeout){
        this.trieurSubsystem = trieurSubsystem;
        this.timeout = timeout;
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
        timeout -= 1;
    }

    @Override
    public boolean isFinished(){
        return timeout <= 0;
    }
}
