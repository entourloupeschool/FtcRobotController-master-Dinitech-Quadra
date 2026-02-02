package org.firstinspires.ftc.teamcode.dinitech.commands.modes;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.AnywhereToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.CancelFollowPath;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeShootSuper extends ConditionalCommand {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param drivePedroSubsystem    The drive subsystem for driving the robot.
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param shooterSubsystem  The shooter subsystem for running the shooter motor.
     */
    public ModeShootSuper(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem,
                          GamepadSubsystem gamepadSubsystem, CommandBase commandBase, boolean team) {
        super(
                new AnywhereToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, commandBase, team),
                new ConditionalCommand(
                        new CancelFollowPath(drivePedroSubsystem),
                        new InstantCommand(),
                        () -> {
                            return drivePedroSubsystem.getDrive().isOnPath();
                        }
                ),
                () -> {
                    boolean driverInputPose = drivePedroSubsystem.getDriverInputPose();
                    boolean driveIsAllreadyOnPath = drivePedroSubsystem.getDrive().isOnPath();
                    boolean driverInputAtRest = gamepadSubsystem.getDriver().getGamepadEx().gamepad.atRest();

                    return driverInputPose && !driveIsAllreadyOnPath && driverInputAtRest;
                }
        );
    }

    public ModeShootSuper(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, GamepadSubsystem gamepadSubsystem, boolean team) {
        super(
                new AnywhereToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, team),
                new ConditionalCommand(
                        new CancelFollowPath(drivePedroSubsystem),
                        new InstantCommand(),
                        () -> {
                            return drivePedroSubsystem.getDrive().isOnPath();
                        }
                ),
                () -> {
                    boolean driverInputPose = drivePedroSubsystem.getDriverInputPose();
                    boolean driveIsAllreadyOnPath = drivePedroSubsystem.getDrive().isOnPath();
                    boolean driverInputAtRest = gamepadSubsystem.getDriver().getGamepadEx().gamepad.atRest();

                    return driverInputPose && !driveIsAllreadyOnPath && driverInputAtRest;
                }
        );
    }
}
