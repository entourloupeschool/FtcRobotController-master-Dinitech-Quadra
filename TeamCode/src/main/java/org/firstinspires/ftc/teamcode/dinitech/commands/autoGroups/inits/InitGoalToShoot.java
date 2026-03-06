package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_SHOOT_POSE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitGoalToShoot extends SequentialCommandGroup {

    public InitGoalToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, HubsSubsystem hubsSubsystem){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(),
                                new SetVelocityShooterRequire(shooterSubsystem,CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY)),

                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierLine(
                                        drivePedroSubsystem::getPose,
                                        hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? (drivePedroSubsystem.getPose().getY() > 72 ? CLOSE_SHOOT_BLUE_POSE : BLUE_AUDIENCE_SHOOT_POSE) : (drivePedroSubsystem.getPose().getY() > 72 ? CLOSE_SHOOT_RED_POSE : RED_AUDIENCE_SHOOT_POSE))
                                ).setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        (hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? (drivePedroSubsystem.getPose().getY() > 72 ? CLOSE_SHOOT_BLUE_POSE : BLUE_AUDIENCE_SHOOT_POSE) : (drivePedroSubsystem.getPose().getY() > 72 ? CLOSE_SHOOT_RED_POSE : RED_AUDIENCE_SHOOT_POSE)).getHeading(),
                                        LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                                AUTO_ROBOT_CONSTRAINTS, true)),

                new ShootHighSpeedIntel(trieurSubsystem, shooterSubsystem)
        );
    }

}
