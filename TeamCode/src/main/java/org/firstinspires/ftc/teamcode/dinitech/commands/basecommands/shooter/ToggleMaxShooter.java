package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class ToggleMaxShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ToggleMaxShooter(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        // Cancel any currently running command that uses the shooter subsystem
        Command currentCommand = CommandScheduler.getInstance().requiring(shooterSubsystem);
        if (currentCommand != null) {
            currentCommand.cancel();
        }

        // Toggle based on actual shooter state
        if (shooterSubsystem.isPowered()) {
            // Shooter is running, so stop it
            new StopShooter(shooterSubsystem).schedule();
        } else {
            // Shooter is stopped, so start it
            new MaxSpeedShooter(shooterSubsystem).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately after scheduling the appropriate command
        return true;
    }
}