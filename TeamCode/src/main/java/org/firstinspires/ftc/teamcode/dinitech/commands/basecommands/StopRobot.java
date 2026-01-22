package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

/**
 * A Sequential Command Group that stops the robot.
 */
public class StopRobot extends SequentialCommandGroup {
    public StopRobot(DrivePedroSubsystem drivePedroSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem){
        addCommands(
                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new StopShooter(shooterSubsystem)
                ),
                new InstantCommand(
                        () -> {
                            CommandScheduler.getInstance().cancelAll();}
                ),
                new InstantCommand(
                        drivePedroSubsystem::stopAllMotors
                )
        );

        addRequirements(drivePedroSubsystem, chargeurSubsystem, shooterSubsystem);
    }
}
