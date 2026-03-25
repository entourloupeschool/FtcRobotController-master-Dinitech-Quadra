package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_BLUE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_RED;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueTestResetPoseAutoBase;

@Autonomous(name = "TestOptimalCurve - Dinitech", group = "Test")
//@Disabled
public class TestOptimalCurve extends BlueTestResetPoseAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new SequentialCommandGroup(
                    OptimalPath.curve(drivePedroSubsystem, CLOSE_SHOOT_RED_POSE, hubsSubsystem.getTeam().getCloseShootPose(), AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.curve(drivePedroSubsystem, FIELD_CENTER_90HEADING_POSE, RESET_POSE_RED, AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.line(drivePedroSubsystem, hubsSubsystem.getTeam().getAudienceShootPose(),  AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.curve(drivePedroSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), CLOSE_SHOOT_RED_POSE, AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(1000),
                    OptimalPath.curve(drivePedroSubsystem, FIELD_CENTER_90HEADING_POSE, RESET_POSE_BLUE, AUTO_ROBOT_CONSTRAINTS, true)
            ).schedule();
    }

}
