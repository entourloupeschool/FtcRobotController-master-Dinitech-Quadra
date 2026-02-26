package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAlmostRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class AnywhereToShoot extends SequentialCommandGroup {
    public AnywhereToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, CommandBase commandBase, boolean team){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY : SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY),
                        new SequentialCommandGroup(
                                commandBase,
                                new OpenWaitTrappe(trieurSubsystem)
                        ),
                        new ConditionalCommand(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_BLUE_POSE : BLUE_SMALL_TRIANGLE_SHOOT_POSE)
                                        ).setLinearHeadingInterpolation(drivePedroSubsystem.getPose().getHeading(), drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_BLUE_POSE.getHeading() : BLUE_SMALL_TRIANGLE_SHOOT_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_RED_POSE : RED_SMALL_TRIANGLE_SHOOT_POSE)
                                        ).setLinearHeadingInterpolation(drivePedroSubsystem.getPose().getHeading(), drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_RED_POSE.getHeading() : RED_SMALL_TRIANGLE_SHOOT_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                () -> team)),

                new ShootHighSpeedRevolution(trieurSubsystem)
        );
    }

    public AnywhereToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, boolean team){
        addCommands(
                new ParallelCommandGroup(
                        new SetVelocityShooter(shooterSubsystem, drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY : SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY),
                        new OpenWaitTrappe(trieurSubsystem),
                        new ConditionalCommand(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_BLUE_POSE : BLUE_SMALL_TRIANGLE_SHOOT_POSE)
                                        ).setLinearHeadingInterpolation(drivePedroSubsystem.getPose().getHeading(), drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_BLUE_POSE.getHeading() : BLUE_SMALL_TRIANGLE_SHOOT_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_RED_POSE : RED_SMALL_TRIANGLE_SHOOT_POSE)
                                        ).setLinearHeadingInterpolation(drivePedroSubsystem.getPose().getHeading(), drivePedroSubsystem.getPose().getY() > 70 ? CLOSE_SHOOT_RED_POSE.getHeading() : RED_SMALL_TRIANGLE_SHOOT_POSE.getHeading(), LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                () -> team)),

                new ShootAlmostRevolution(trieurSubsystem)
        );
    }
}
