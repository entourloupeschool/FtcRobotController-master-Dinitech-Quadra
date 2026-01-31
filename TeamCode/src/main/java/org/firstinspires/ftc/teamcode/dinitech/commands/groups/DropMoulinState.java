package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ClearMoulinShootingPos;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinToPosition;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that shoots an artifact from a specific moulin position.
 * <p>
 * This command executes a sequence to aim and fire from a designated shooting position:
 * <ol>
 *     <li>Spins up the shooter motor and simultaneously rotates the moulin to the target position
 *     (in parallel).</li>
 *     <li>Opens the trappe to allow the artifact to enter the shooter.</li>
 *     <li>Clears the internal state of the moulin for the now-empty slot.</li>
 * </ol>
 * This class provides a constructor for default shooter behavior (max speed) and a protected
 * constructor for more advanced use cases, such as vision-based shooter speed control.
 */
public class DropMoulinState extends SequentialCommandGroup {

    /**
     * Creates a new DropMoulinState command.
     *
     * @param trieurSubsystem       The sorter subsystem.
     * @param shooterSubsystem      The shooter subsystem.
     * @param moulinPositionToShoot The target shooting position.
     */
    public DropMoulinState(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem,
                           int moulinPositionToShoot, boolean makeShort) {
        addCommands(
            new MoulinToPosition(trieurSubsystem, moulinPositionToShoot, makeShort),
            new WaitUntilCommand(shooterSubsystem::isTargetSpeedStabilized),
            // Once both are ready, open the trappe to shoot
            new OpenTrappe(trieurSubsystem),
            // Finally, update the sorter's internal state
            new ClearMoulinShootingPos(trieurSubsystem, moulinPositionToShoot)
        );
    }
}
