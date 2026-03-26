package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class RampEnd extends ParallelCommandGroup {
    public RampEnd(DrivePedroSubsystem drivePedroSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, Pose rampPose){
        addCommands(
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                rampPose.withX(rampPose.getX() + (rampPose.getX() > 72 ? -8 : 8))))
                        .setLinearHeadingInterpolation(
                                drivePedroSubsystem.getPose().getHeading(),
                                rampPose.getHeading(),
                                LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                        AUTO_ROBOT_CONSTRAINTS, true),
                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new StopShooter(shooterSubsystem))
        );
    }
}
