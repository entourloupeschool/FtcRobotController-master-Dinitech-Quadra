package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;


import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.UntilMotifDetection;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class RaceLookMotifPath extends ParallelRaceGroup {

    public RaceLookMotifPath(DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem, Pose lookMotifPose){
        super(
                OptimalPath.line(drivePedroSubsystem,
                        lookMotifPose, 1, true),
                new UntilMotifDetection(visionSubsystem)
        );
    }
}
