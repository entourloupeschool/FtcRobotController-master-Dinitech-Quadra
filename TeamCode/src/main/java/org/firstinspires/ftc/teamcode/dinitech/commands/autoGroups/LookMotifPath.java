package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

public class LookMotifPath extends SequentialCommandGroup {
    public LookMotifPath(DrivePedroSubsystem drivePedroSubsystem, Pose lookMotifPose){
        addCommands(
                OptimalPath.line(drivePedroSubsystem, lookMotifPose, 1, true),
                new WaitCommand(550)

        );
    }
}
