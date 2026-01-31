package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToggleUsageStateShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    public ToggleUsageStateShooter(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
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
        if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.VISION) {
            shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.NONE);
            new StopShooter(shooterSubsystem).schedule();
            shooterSubsystem.setDefaultCommand(new RunCommand(
                    () -> {},
                    shooterSubsystem
            ));

        } else if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.TELE){
            shooterSubsystem.setDefaultCommand(new VisionShooter(shooterSubsystem, visionSubsystem));

        } else {
            shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));
        }
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately after scheduling the appropriate command
        return true;
    }
}