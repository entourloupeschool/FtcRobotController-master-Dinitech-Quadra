package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_OPEN_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ClearMoulinShootingPos;
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

    /**
     * A protected constructor for subclasses to provide a custom shooter command.
     * <p>
     * This allows replacing the default {@link MaxSpeedShooter} behavior with a different
     * shooting logic, such as a velocity determined by computer vision.
     *
     * @param trieurSubsystem  The sorter subsystem.
     * @param shooterCommand   The custom command to run for controlling the shooter.
     */
    public ShootRevolution(TrieurSubsystem trieurSubsystem, Command shooterCommand) {
        addCommands(
                shooterCommand, // Rev up the shooter
                new OpenTrappe(trieurSubsystem), // Open the trapdoor
                new WaitCommand(TRAPPE_OPEN_TIME), // Wait for the trappe to open fully
                new MoulinRevolution(trieurSubsystem), // Perform a full revolution
                new InstantCommand(trieurSubsystem::clearAllStoredColors)
        );
    }
}
