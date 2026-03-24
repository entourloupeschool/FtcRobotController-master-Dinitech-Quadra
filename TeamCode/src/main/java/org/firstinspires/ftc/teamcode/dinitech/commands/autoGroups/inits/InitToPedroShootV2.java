package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INIT_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_INIT_PEDRO_SHOOTER;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.InstantPedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitToPedroShootV2 extends ParallelCommandGroup {

    public InitToPedroShootV2(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, HubsSubsystem hubsSubsystem, Pose ShootPosition, double endTime, double scaleBrakingStrength){
        final double startRangeToShootPose = drivePedroSubsystem.getPose().distanceFrom(ShootPosition);
        addCommands(
                new SequentialCommandGroup(
                        new SetVelocityShooterRequire(shooterSubsystem, INIT_SHOOT_AUTO_SHOOTER_VELOCITY),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(WAIT_INIT_PEDRO_SHOOTER),
                                        new ShootAll(trieurSubsystem, shooterSubsystem, true)),
                                new RepeatCommand(new InstantPedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem))
                                        .interruptOn(trieurSubsystem::isEmpty))),

                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                ShootPosition))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                ShootPosition.getHeading(),
                                endTime))
                        .setBrakingStrength(scaleBrakingStrength).build(),
                        AUTO_ROBOT_CONSTRAINTS, true)
        );
    }
}
