package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class RampEnd extends ParallelCommandGroup {
    public RampEnd(DrivePedroSubsystem drivePedroSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, Pose rampPose){
        addCommands(
                OptimalPath.line(drivePedroSubsystem,
                        rampPose.withX(rampPose.getX() + (rampPose.getX() > 72 ? -8 : 8)), 1, true),
                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new StopShooter(shooterSubsystem))
        );
    }
}
