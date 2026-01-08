package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;

public class ContinuousRumbleCustom extends CommandBase {
    private final GamepadSubsystem gamepadSubsystem;
    private final int gamepadNumber;
    private final double power;
    private Gamepad.RumbleEffect customRumbleEffect;

    public ContinuousRumbleCustom(GamepadSubsystem gamepadSubsystem, int gamepadNumber, double power) {
        this.gamepadSubsystem = gamepadSubsystem;
        this.gamepadNumber = gamepadNumber;
        this.power = power;
    }

    @Override
    public void initialize() {
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(power, power, 10)
                .build();
    }

    @Override
    public void execute() {
        gamepadSubsystem.customRumble(customRumbleEffect, gamepadNumber);
    }
}
