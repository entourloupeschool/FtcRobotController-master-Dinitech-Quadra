package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class DefaultGamepadCommand extends CommandBase {
    private final GamepadSubsystem gamepadSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final TrieurSubsystem trieurSubsystem;

    public DefaultGamepadCommand(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, GamepadSubsystem gamepadSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        addRequirements(gamepadSubsystem);
    }

    @Override
    public void execute() {
        double shooterPower = shooterSubsystem.getPower();
        Gamepad.RumbleEffect customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0, shooterPower, 10)
                .build();
        gamepadSubsystem.customRumble(customRumbleEffect, 2);
    }
}
