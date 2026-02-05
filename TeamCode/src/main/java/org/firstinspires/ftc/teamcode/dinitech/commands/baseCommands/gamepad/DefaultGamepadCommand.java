package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_DURATION_1;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_DURATION_3;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class DefaultGamepadCommand extends CommandBase {
    private final GamepadSubsystem gamepadSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final TrieurSubsystem trieurSubsystem;

    private Gamepad.RumbleEffect trappeOpenRumble;
    private Gamepad.RumbleEffect shooterVelocityTargetStabilizedRumble;

    private Gamepad.RumbleEffect aimLockedDriveRumble;



    public DefaultGamepadCommand(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, GamepadSubsystem gamepadSubsystem){
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        addRequirements(gamepadSubsystem);
    }

    @Override
    public void initialize(){
        trappeOpenRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.35, 0, RUMBLE_DURATION_1)
                .build();

        shooterVelocityTargetStabilizedRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 0.15, RUMBLE_DURATION_3)
                .addStep(0, 0.25, RUMBLE_DURATION_3)
                .addStep(0.15, 0.35, RUMBLE_DURATION_3)
                .addStep(0.25, 0.45, RUMBLE_DURATION_3)
                .addStep(0.35, 0.55, RUMBLE_DURATION_3)
                .addStep(0.45, 0.65, RUMBLE_DURATION_3)
                .addStep(0.55, 0.55, RUMBLE_DURATION_3)
                .addStep(0.65, 0.45, RUMBLE_DURATION_3)
                .addStep(0.55, 0.35, RUMBLE_DURATION_3)
                .addStep(0.45, 0.25, RUMBLE_DURATION_3)
                .addStep(0.35, 0.15, RUMBLE_DURATION_3)
                .addStep(0.25, 0, RUMBLE_DURATION_3)
                .addStep(0.15, 0, RUMBLE_DURATION_3)
                .build();

        aimLockedDriveRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 0.15, RUMBLE_DURATION_3)
                .addStep(0.15, 0.30, RUMBLE_DURATION_3)
                .addStep(0.30, 0.45, RUMBLE_DURATION_3)
                .addStep(0.45, 0.30, RUMBLE_DURATION_3)
                .addStep(0.30, 0.15, RUMBLE_DURATION_3)
                .addStep(0.15, 0, RUMBLE_DURATION_3)
                .build();
    }

    @Override
    public void execute() {
        if (shooterSubsystem.getVelocity() > 20 && shooterSubsystem.isTargetSpeedStabilized() && !gamepadSubsystem.isRumbling(2)){
            gamepadSubsystem.customRumble(shooterVelocityTargetStabilizedRumble, 2, true);
        }

//        if (trieurSubsystem.isTrappeOpen() && !gamepadSubsystem.isRumbling(3)){
//            gamepadSubsystem.customRumble(trappeOpenRumble, 3, true);
//        }

        boolean isAiming = drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.PEDRO_AIM || drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.VISION_AIM;

        if (isAiming && !gamepadSubsystem.isRumbling(1)){
            gamepadSubsystem.customRumble(aimLockedDriveRumble, 1, true);
        }

    }
}
