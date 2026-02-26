package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.function.Supplier;

public class SwitchUsageStateShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    private final HubsSubsystem hubsSubsystem;


    public SwitchUsageStateShooter(ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        this.hubsSubsystem = hubsSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Toggle based on actual shooter state
        if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.VISION) {
            shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.STOP);
            new StopShooter(shooterSubsystem).schedule();
            shooterSubsystem.setDefaultCommand(new RunCommand(
                    () -> {},
                    shooterSubsystem
            ));

        } else if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.STOP){
            shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));
        } else if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.TELE){
            shooterSubsystem.setDefaultCommand(new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem));
        } else if (shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.PEDRO) {
            shooterSubsystem.setDefaultCommand(new VisionShooter(shooterSubsystem, visionSubsystem));
        } else {
            shooterSubsystem.setDefaultCommand(new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem));
        }
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately after scheduling the appropriate command
        return true;
    }
}