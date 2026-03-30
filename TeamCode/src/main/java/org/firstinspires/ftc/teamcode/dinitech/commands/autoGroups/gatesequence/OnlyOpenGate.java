package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

public class OnlyOpenGate extends SequentialCommandGroup {
    public OnlyOpenGate(DrivePedroSubsystem drivePedroSubsystem, Pose gatePickPose){
        addCommands(
                OptimalPath.curve(
                        drivePedroSubsystem,
                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -1.5*TILE_DIM : 1.5*TILE_DIM)),
                        gatePickPose, 1, true),

                OptimalPath.line(drivePedroSubsystem,
                        gatePickPose.withHeading(Math.abs(gatePickPose.getHeading()) < Math.PI/2 ? 0 : Math.PI), 1, true),
                new WaitCommand(5000),
                OptimalPath.line(drivePedroSubsystem,
                        gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -1.7*TILE_DIM : 1.7*TILE_DIM)), 1, true)

        );
    }
}
