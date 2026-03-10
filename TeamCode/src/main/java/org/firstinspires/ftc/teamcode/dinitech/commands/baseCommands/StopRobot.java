package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A Sequential Command Group that stops the robot.
 */
public class StopRobot extends SequentialCommandGroup {
    public StopRobot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem){
        addCommands(
                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new StopShooter(shooterSubsystem),
                        new InstantCommand(trieurSubsystem::resetTargetMoulinMotor, trieurSubsystem)),
                new InstantCommand(() -> {CommandScheduler.getInstance().cancelAll();}),
                new InstantCommand(drivePedroSubsystem::stopAllMotors, drivePedroSubsystem)
        );

        addRequirements(drivePedroSubsystem, trieurSubsystem, chargeurSubsystem, shooterSubsystem);
    }
}
