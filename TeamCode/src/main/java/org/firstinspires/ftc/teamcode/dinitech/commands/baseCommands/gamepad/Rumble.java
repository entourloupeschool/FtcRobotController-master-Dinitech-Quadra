package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;

public class Rumble extends CommandBase {
    private final GamepadSubsystem gamepadSubsystem;
    private final int gamepadNumber;
    private final int rumbleEffectNumber;

    public Rumble(GamepadSubsystem gamepadSubsystem, int gamepadNumber, int rumbleEffectNumber) {
        this.gamepadSubsystem = gamepadSubsystem;
        this.gamepadNumber = gamepadNumber;
        this.rumbleEffectNumber = rumbleEffectNumber;
    }

    @Override
    public void initialize() {
        switch (gamepadNumber) {
            case 1:
                gamepadSubsystem.driverRumble(rumbleEffectNumber);
            case 2:
                gamepadSubsystem.operatorRumble(rumbleEffectNumber);
            case 3:
                gamepadSubsystem.driverRumble(rumbleEffectNumber);
                gamepadSubsystem.operatorRumble(rumbleEffectNumber);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
