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
        if (shooterSubsystem.getVelocity() > 10){
            gamepadSubsystem.customRumble(new Gamepad.RumbleEffect.Builder()
                    .addStep(0, shooterSubsystem.isTargetSpeedStabilized() ? 0.15 : 0, RUMBLE_DURATION_1)
                    .addStep(0, shooterSubsystem.isTargetSpeedStabilized() ? 0.25 : 0, RUMBLE_DURATION_1)
                    .build(), 2);
        }

        double trappe = trieurSubsystem.isTrappeOpen() ? 0.15 : 0;
        if (trappe > 0){
            gamepadSubsystem.customRumble(new Gamepad.RumbleEffect.Builder()
                    .addStep(trappe, 0, RUMBLE_DURATION_1)
                    .build(), 1);
        }

    }
}
