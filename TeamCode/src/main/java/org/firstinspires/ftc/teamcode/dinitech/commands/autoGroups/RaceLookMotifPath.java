package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OBELISK_POSE;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.UntilMotifDetection;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class RaceLookMotifPath extends ParallelRaceGroup {

    public RaceLookMotifPath(DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem, Pose lookMotifPose, double scaleBrakingStrength, double endTime){
        super(
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                lookMotifPose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                lookMotifPose.getHeading(),
                                endTime))
                        .setBrakingStrength(scaleBrakingStrength).build(),
                        AUTO_ROBOT_CONSTRAINTS, true),
                    new UntilMotifDetection(visionSubsystem)
        );
    }
    public RaceLookMotifPath(DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem, Pose lookMotifPose){
        super(
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierLine(
                                drivePedroSubsystem::getPose,
                                lookMotifPose))
                        .setHeadingInterpolation(HeadingInterpolator.facingPoint(OBELISK_POSE)).build(),
                        AUTO_ROBOT_CONSTRAINTS, true),
                new UntilMotifDetection(visionSubsystem)
        );
    }
}
