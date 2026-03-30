package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.WAIT_INIT_SHOOTER;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitToQuickShoot extends ParallelCommandGroup {

    public InitToQuickShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, Pose shootPose, double shootVelocity){
        addCommands(
                new SequentialCommandGroup(
                        new InstantCommand(),
                        new ParallelCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new SetVelocityShooterRequire(shooterSubsystem, shootVelocity)),
                        new WaitCommand(WAIT_INIT_SHOOTER),
                        new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true)),
                OptimalPath.line(drivePedroSubsystem, shootPose, 1, true)

        );
    }
}
