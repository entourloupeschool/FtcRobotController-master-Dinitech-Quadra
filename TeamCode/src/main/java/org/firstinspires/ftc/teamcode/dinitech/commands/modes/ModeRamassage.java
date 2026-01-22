package org.firstinspires.ftc.teamcode.dinitech.commands.modes;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassage extends ConditionalCommand {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param shooterSubsystem  The shooter subsystem for running the shooter motor.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassage(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem,
                         GamepadSubsystem gamepadSubsystem) {
        super(
                // if condition is true.
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new CloseTrappe(trieurSubsystem),
                                new StopShooter(shooterSubsystem)
                        ),
                        new AutomaticArtefactPickAway(trieurSubsystem, chargeurSubsystem, gamepadSubsystem)),

                // if condition is false.
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new CloseTrappe(trieurSubsystem),
                            new StopShooter(shooterSubsystem)
                    ),
                    new MoulinNext(trieurSubsystem),
                    new AutomaticArtefactPickAway(trieurSubsystem, chargeurSubsystem, gamepadSubsystem)),
                
                // Condition.
                () -> Moulin.isStoragePosition(trieurSubsystem.getMoulinPosition())
        );

    }
}
