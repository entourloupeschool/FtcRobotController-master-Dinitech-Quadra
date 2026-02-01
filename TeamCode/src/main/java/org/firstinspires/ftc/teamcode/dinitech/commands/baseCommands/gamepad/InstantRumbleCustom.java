package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;

public class InstantRumbleCustom extends CommandBase {
    private final GamepadSubsystem gamepadSubsystem;
    private final int gamepadNumber;
    private final double power;

    public InstantRumbleCustom(GamepadSubsystem gamepadSubsystem, int gamepadNumber, double power) {
        this.gamepadSubsystem = gamepadSubsystem;
        this.gamepadNumber = gamepadNumber;
        this.power = power;
    }

    @Override
    public void initialize() {
        Gamepad.RumbleEffect customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(power, power, 10)
                .build();
        gamepadSubsystem.customRumble(customRumbleEffect, gamepadNumber, true);
    }
}
