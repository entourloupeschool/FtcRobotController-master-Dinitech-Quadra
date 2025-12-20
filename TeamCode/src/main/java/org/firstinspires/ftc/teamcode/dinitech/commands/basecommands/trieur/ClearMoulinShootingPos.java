package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * An instant command that updates the sorter's internal state after an artifact has been shot.
 * <p>
 * This command takes a shooting position as input, determines the corresponding storage slot,
 * and clears the color data for that slot in the {@link TrieurSubsystem}. This effectively
 * marks the slot as empty, ensuring the robot's knowledge base is accurate.
 */
public class ClearMoulinShootingPos extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final int shootingPos;

    /**
     * Creates a new ClearMoulinShootingPos command.
     *
     * @param trieurSubsystem The sorter subsystem to update.
     * @param shootingPos     The moulin position from which an artifact was just shot.
     */
    public ClearMoulinShootingPos(TrieurSubsystem trieurSubsystem, int shootingPos){
        this.trieurSubsystem = trieurSubsystem;
        this.shootingPos = shootingPos;
    }

    /**
     * Executes the logic to clear the storage slot.
     *
     * @throws IllegalArgumentException if the provided position is not a valid shooting position.
     */
    @Override
    public void initialize(){
        if (!Moulin.isShootingPosition(shootingPos)) {
            throw new IllegalArgumentException("Invalid shooting position provided to ClearMoulinShootingPos: " + shootingPos);
        }

        // Determine which storage slot corresponds to the shooting position and clear it.
        int storagePosToClear = Moulin.getStoragePositionFromShootingPosition(shootingPos);
        trieurSubsystem.clearMoulinStoragePositionColor(storagePosToClear);
        trieurSubsystem.setIsFull(false);
    }

    /**
     * This command finishes immediately after execution.
     *
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
