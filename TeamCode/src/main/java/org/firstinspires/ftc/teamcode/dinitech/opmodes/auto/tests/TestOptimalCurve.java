package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_BLUE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_RED;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueTestResetPoseAutoBase;

@Autonomous(name = "TestOptimalCurveV2 - Dinitech", group = "Test")
//@Disabled
public class TestOptimalCurveV2 extends BlueTestResetPoseAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new SequentialCommandGroup(
                    OptimalPath.cubic(drivePedroSubsystem, RESET_POSE_RED, CLOSE_SHOOT_RED_POSE, hubsSubsystem.getTeam().getCloseShootPose(),  AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.cubic(drivePedroSubsystem, CLOSE_SHOOT_RED_POSE, RESET_POSE_BLUE, RESET_POSE_RED, AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.cubic(drivePedroSubsystem, CLOSE_SHOOT_RED_POSE, RESET_POSE_BLUE, hubsSubsystem.getTeam().getAudienceShootPose(),  AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.cubic(drivePedroSubsystem, RESET_POSE_BLUE, hubsSubsystem.getTeam().getCloseShootPose(), CLOSE_SHOOT_RED_POSE, AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.cubic(drivePedroSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), RESET_POSE_RED, RESET_POSE_BLUE, AUTO_ROBOT_CONSTRAINTS, true)
            ).schedule();
    }

}
