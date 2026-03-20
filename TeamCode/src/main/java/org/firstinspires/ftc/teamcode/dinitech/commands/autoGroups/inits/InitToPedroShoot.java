package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INIT_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_INIT_PEDRO_SHOOTER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_INIT_SHOOTER;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShootWaitVelocity;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.WaitShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitToPedroShoot extends ParallelCommandGroup {

    public InitToPedroShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, Pose ShootPosition, double shootVelocity, double endTime, double scaleBrakingStrength){
        final double startRangeToShootPose = drivePedroSubsystem.getPose().distanceFrom(ShootPosition);
        addCommands(
                new SequentialCommandGroup(
                        new SetVelocityShooterRequire(shooterSubsystem, INIT_SHOOT_AUTO_SHOOTER_VELOCITY),
                        new WaitCommand(WAIT_INIT_PEDRO_SHOOTER),
                        new ShootAll(trieurSubsystem, shooterSubsystem)),

                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                ShootPosition))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                ShootPosition.getHeading(),
                                endTime))
                        .setBrakingStrength(scaleBrakingStrength).build(),
                        AUTO_ROBOT_CONSTRAINTS, true,
                        () -> {
                            if (startRangeToShootPose <= 1e-6) {
                                shooterSubsystem.setVelocity(shootVelocity);
                                return;
                            }

                            double remainingRangeToShootPose = drivePedroSubsystem.getPose().distanceFrom(ShootPosition);
                            double t = 1.0 - (remainingRangeToShootPose / startRangeToShootPose);
                            double clampedT = Math.max(0.0, Math.min(1.0, t));

                            double targetVelocity = INIT_SHOOT_AUTO_SHOOTER_VELOCITY
                                    + (shootVelocity - INIT_SHOOT_AUTO_SHOOTER_VELOCITY) * clampedT;
                            shooterSubsystem.setVelocity(targetVelocity);
                        })
        );
    }
}
