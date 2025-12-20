package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that performs a full revolution of the moulin to shoot all loaded artifacts.
 * <p>
 * This command orchestrates a complete shooting sequence:
 * <ol>
 *     <li>Spins up the shooter to maximum speed.</li>
 *     <li>Opens the trappe (trapdoor).</li>
 *     <li>Performs a full revolution of the moulin to feed all artifacts.</li>
 *     <li>Closes the trappe and stops the shooter motor simultaneously.</li>
 * </ol>
 * It also includes cleanup logic to reset the state of the subsystems upon completion or interruption.
 */
public class ShootRevolution extends SequentialCommandGroup {

    private final TrieurSubsystem trieurSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates a new ShootRevolution command with default behavior.
     *
     * @param trieurSubsystem  The sorter subsystem that controls the moulin and trappe.
     * @param shooterSubsystem The shooter subsystem.
     */
    public ShootRevolution(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        this(trieurSubsystem, shooterSubsystem, new MaxSpeedShooter(shooterSubsystem));
    }

    /**
     * A protected constructor for subclasses to provide a custom shooter command.
     * <p>
     * This allows replacing the default {@link MaxSpeedShooter} behavior with a different
     * shooting logic, such as a velocity determined by computer vision.
     *
     * @param trieurSubsystem  The sorter subsystem.
     * @param shooterSubsystem The shooter subsystem.
     * @param shooterCommand   The custom command to run for controlling the shooter.
     */
    protected ShootRevolution(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, Command shooterCommand) {
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addCommands(
                shooterCommand, // Rev up the shooter
                new OpenTrappe(trieurSubsystem), // Open the trapdoor
                new WaitCommand(200), // Wait for the trappe to open fully
                new MoulinRevolution(trieurSubsystem), // Perform a full revolution
                new WaitCommand(100), // Wait for the last artifact to clear
                new ParallelCommandGroup( // Clean up
                        new CloseTrappe(trieurSubsystem),
                        new StopShooter(shooterSubsystem)
                )
        );
    }

    /**
     * Cleans up the subsystems at the end of the command.
     * <p>
     * This method clears the stored artifact colors from the sorter. If the command
     * was interrupted, it also ensures the shooter is stopped and the trappe is closed
     * as a safety measure.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        // Always clear the artifact knowledge base after a full revolution
        trieurSubsystem.clearAllStoredColors();

        // If the command was cancelled, perform immediate cleanup for safety
        if (interrupted) {
            shooterSubsystem.stopMotor();
            trieurSubsystem.closeTrappe();
        }
    }
}
