package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ClearMoulinShootingPos;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinToPosition;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
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
public class ShootMoulinState extends SequentialCommandGroup {

    /**
     * Creates a new ShootMoulinState command with a custom shooter command.
     * <p>
     * This protected constructor is intended for subclasses that need to provide a different
     * shooter behavior, such as a variable speed determined by computer vision.
     *
     * @param trieurSubsystem       The sorter subsystem.
     * @param shooterSubsystem      The shooter subsystem.
     * @param moulinPositionToShoot The target shooting position.
     * @param shooterCommand        A custom {@link Command} to control the shooter speed.
     */
    public ShootMoulinState(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem,
            int moulinPositionToShoot, Command shooterCommand) {
        addCommands(
                // Spin up the shooter and rotate the moulin at the same time
                new ParallelCommandGroup(
                        shooterCommand,
                        new MoulinToPosition(trieurSubsystem, moulinPositionToShoot, false)
                ),
                // Once both are ready, open the trappe to shoot
                new OpenTrappe(trieurSubsystem),
                // Finally, update the sorter's internal state
                new ClearMoulinShootingPos(trieurSubsystem, moulinPositionToShoot)
        );
    }
}
