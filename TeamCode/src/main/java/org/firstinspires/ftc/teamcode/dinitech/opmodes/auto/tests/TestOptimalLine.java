package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.RESET_POSE_RED;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.RESET_POSE_BLUE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueTestResetPoseAutoBase;

@Autonomous(name = "TestOptimalLine - Dinitech", group = "Test")
//@Disabled
public class TestOptimalLine extends BlueTestResetPoseAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new SequentialCommandGroup(
                    OptimalPath.line(drivePedroSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), 1, true),
                    new WaitCommand(1000),
                    OptimalPath.line(drivePedroSubsystem, RESET_POSE_RED, 1, true),
                    new WaitCommand(1000),
                    OptimalPath.line(drivePedroSubsystem, hubsSubsystem.getTeam().getAudienceShootPose(), 1, true),
                    new WaitCommand(1000),
                    OptimalPath.line(drivePedroSubsystem, CLOSE_SHOOT_RED_POSE, 1, true),
                    new WaitCommand(1000),
                    OptimalPath.line(drivePedroSubsystem, RESET_POSE_BLUE, 1, true)
            ).schedule();
    }
}
