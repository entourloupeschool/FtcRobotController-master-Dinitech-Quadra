package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_DURATION_1;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_DURATION_3;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

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





    public DefaultGamepadCommand(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, GamepadSubsystem gamepadSubsystem){
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
        addRequirements(gamepadSubsystem);
    }

    @Override
    public void execute() {
        if (trieurSubsystem.isMoulinOverCurrent()){gamepadSubsystem.customRumble(gamepadSubsystem.moulinOverCurrentRumble, 3, true);}
        else {
            if (shooterSubsystem.getVelocity() > 20 && shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN) && !gamepadSubsystem.isRumbling(2)){
                gamepadSubsystem.customRumble(gamepadSubsystem.shooterVelocityTargetStabilizedRumble, 2, true);
            }

            boolean isAiming = drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.PEDRO_AIM || drivePedroSubsystem.getDriveAimLockType() == DrivePedroSubsystem.DriveAimLockType.VISION_AIM;

            if (isAiming && !gamepadSubsystem.isRumbling(1)){
                gamepadSubsystem.customRumble(gamepadSubsystem.aimLockedDriveRumble, 1, true);
            }
        }
    }
}
