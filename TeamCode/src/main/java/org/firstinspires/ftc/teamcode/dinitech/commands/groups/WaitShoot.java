package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.MaxSpeedShooter;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;


public class WaitShoot extends SequentialCommandGroup {
    public WaitShoot(ShooterSubsystem shooterSubsystem, TrieurSubsystem trieurSubsystem) {
        addCommands(
                new ParallelRaceGroup(
                        new WaitUntilCommand(shooterSubsystem::isCurrentOverflow),
                        new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)),
                new ConditionalCommand(
                        new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                        new InstantCommand(),
                        trieurSubsystem::wantsMotifShoot)
        );
    }
}
