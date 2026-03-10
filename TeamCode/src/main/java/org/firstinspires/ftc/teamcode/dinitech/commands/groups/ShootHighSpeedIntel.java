package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.MaxSpeedShooter;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextShootIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.HashMap;

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
public class ShootHighSpeedIntel extends SelectCommand {

    /**
     * A protected constructor for subclasses to provide a custom shooter command.
     * <p>
     * This allows replacing the default {@link MaxSpeedShooter} behavior with a different
     * shooting logic, such as a velocity determined by computer vision.
     *
     * @param trieurSubsystem  The sorter subsystem.
     */

    public ShootHighSpeedIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        super(
            new HashMap<Object, Command>(){{
                put(0, new InstantCommand());

                put(1, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem)));

                put(2, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem)));

                put(3, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem)));}},

            trieurSubsystem::getHowManyArtefacts
        );
    }
}
