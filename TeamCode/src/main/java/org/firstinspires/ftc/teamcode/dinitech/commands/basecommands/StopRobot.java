package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

/**
 * A Sequential Command Group that stops the robot.
 */
public class StopRobot extends SequentialCommandGroup {
    public StopRobot(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem){
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
                        driveSubsystem::stopAllMotors
                )
        );

        addRequirements(driveSubsystem, chargeurSubsystem, shooterSubsystem);
    }
}
