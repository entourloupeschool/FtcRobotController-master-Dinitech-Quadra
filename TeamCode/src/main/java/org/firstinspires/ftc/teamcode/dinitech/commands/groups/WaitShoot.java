package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.MaxSpeedShooter;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;



public class WaitShoot extends ParallelRaceGroup {

    /**
     * A protected constructor for subclasses to provide a custom shooter command.
     * <p>
     * This allows replacing the default {@link MaxSpeedShooter} behavior with a different
     * shooting logic, such as a velocity determined by computer vision.
     *
     * @param shooterSubsystem  The shooter subsystem.
     */

    public WaitShoot(ShooterSubsystem shooterSubsystem) {
        super(
                new WaitUntilCommand(shooterSubsystem::isCurrentOverflow),
                new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)
        );
    }
}
