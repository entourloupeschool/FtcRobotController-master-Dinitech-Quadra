package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

public class LookMotifPath extends SequentialCommandGroup {
    public LookMotifPath(DrivePedroSubsystem drivePedroSubsystem, Pose lookMotifPose, double scaleBrakingStrength, double endTime){
        addCommands(
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                lookMotifPose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                lookMotifPose.getHeading(),
                                endTime))
                        .setBrakingStrength(scaleBrakingStrength/5).build(),
                        AUTO_ROBOT_CONSTRAINTS, true),
                new WaitCommand(550)

        );
    }
}
