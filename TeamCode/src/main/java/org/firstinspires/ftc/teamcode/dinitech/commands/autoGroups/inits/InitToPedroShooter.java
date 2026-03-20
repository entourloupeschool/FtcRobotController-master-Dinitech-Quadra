package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_START_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_STRENGTH_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INIT_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_INIT_SHOOTER;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitToPedroShooter extends ParallelCommandGroup {

    public InitToPedroShooter(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, Pose ShootPosition, double shootVelocity, double endTime){
        addCommands(
                new SequentialCommandGroup(
                        new SetVelocityShooterRequire(shooterSubsystem, INIT_SHOOT_AUTO_SHOOTER_VELOCITY),
                        new SetVelocityShooterRequire(shooterSubsystem, shootVelocity),
                        new WaitCommand(WAIT_INIT_SHOOTER),
                        new ShootAll(trieurSubsystem, shooterSubsystem, true)),
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                ShootPosition))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                ShootPosition.getHeading(),
                                endTime))
                        .setBrakingStrength(BRAKING_STRENGTH_PEDRO_DINITECH/7).build(),
                        AUTO_ROBOT_CONSTRAINTS, true)
        );
    }
}
