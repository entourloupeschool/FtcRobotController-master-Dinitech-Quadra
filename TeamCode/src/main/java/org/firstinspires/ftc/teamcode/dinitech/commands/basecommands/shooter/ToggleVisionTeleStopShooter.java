package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToggleVisionShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    public ToggleVisionShooter(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    @Override
    public void initialize() {
        // Cancel the current default command to allow a new one to be set.
        if (shooterSubsystem.getDefaultCommand() != null) {
            shooterSubsystem.getDefaultCommand().cancel();
        }


        // Toggle based on actual shooter state
        if (shooterSubsystem.isVisionShooting()) {
            shooterSubsystem.setVisionShooting(false);
            // Shooter is running, so stop it
            new StopShooter(shooterSubsystem).schedule();
        } else {
            new Rumble(gamepadSubsystem, 2, 3).schedule();
            shooterSubsystem.setDefaultCommand(new VisionShooter(shooterSubsystem, visionSubsystem, true));

        }
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately after scheduling the appropriate command
        return true;
    }
}