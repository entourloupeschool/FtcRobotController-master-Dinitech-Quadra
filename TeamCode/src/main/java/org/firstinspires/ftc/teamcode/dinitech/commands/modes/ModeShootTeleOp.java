package org.firstinspires.ftc.teamcode.dinitech.commands.modes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.SetDefault;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.PedroAimLockedDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.PedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeShootTeleOp extends ParallelCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param drivePedroSubsystem    The drive subsystem for driving the robot.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param shooterSubsystem  The shooter subsystem for running the shooter motor.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeShootTeleOp(DrivePedroSubsystem drivePedroSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {
        addCommands(
            new StopChargeur(chargeurSubsystem),
            new SetDefault(shooterSubsystem, new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem)),
            new SetDefault(drivePedroSubsystem, new PedroAimLockedDrive(drivePedroSubsystem, gamepadSubsystem, hubsSubsystem)));
    }
}
