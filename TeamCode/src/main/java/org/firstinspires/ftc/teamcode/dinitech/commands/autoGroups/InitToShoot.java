package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAlmostRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitToShoot extends SequentialCommandGroup {

    public InitToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, Pose ShootPosition, double shooterVelocity){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(),
                                new SetVelocityShooter(shooterSubsystem, shooterVelocity)),

                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        ShootPosition)
                                ).setHeadingInterpolation(HeadingInterpolator.linearFromPoint(drivePedroSubsystem::getHeading, ShootPosition.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)
                ),

                new ShootHighSpeedRevolution(trieurSubsystem)
        );
    }
}
