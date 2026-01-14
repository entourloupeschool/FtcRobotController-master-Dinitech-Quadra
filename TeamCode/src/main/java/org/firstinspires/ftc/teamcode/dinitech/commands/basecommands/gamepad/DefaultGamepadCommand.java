package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_DURATION_1;

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
        double stabilized = shooterSubsystem.isTargetSpeedStabilized() ? 0.2 : 0;
        Gamepad.RumbleEffect customRumbleEffectGp2 = new Gamepad.RumbleEffect.Builder()
                .addStep(0, stabilized, RUMBLE_DURATION_1)
                .build();
        gamepadSubsystem.customRumble(customRumbleEffectGp2, 2);

        // trappeOpen 1 if trieurSubsysem.isTrappeOpen() is true, else 0
        double trappeOpen = trieurSubsystem.isTrappeOpen() ? 0.2 : 0;
        Gamepad.RumbleEffect customRumbleEffectGp1 = new Gamepad.RumbleEffect.Builder()
                .addStep(trappeOpen, 0, RUMBLE_DURATION_1)
                .build();
        gamepadSubsystem.customRumble(customRumbleEffectGp1, 1);
    }
}
