package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class ToggleChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;


    public ToggleChargeur(ChargeurSubsystem chargeurSubsystem) {
        this.chargeurSubsystem = chargeurSubsystem;
    }

    @Override
    public void initialize() {
        // Cancel any currently running command that uses the shooter subsystem
        Command currentCommand = CommandScheduler.getInstance().requiring(chargeurSubsystem);
        if (currentCommand != null) {
            currentCommand.cancel();
        }

        // Toggle based on actual shooter state
        if (chargeurSubsystem.isMotorPowered()) {
            // Shooter is running, so stop it
            new StopChargeur(chargeurSubsystem).schedule();
        } else {
            // Shooter is stopped, so start it
            new MaxPowerChargeur(chargeurSubsystem).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately after scheduling the appropriate command
        return true;
    }
}
