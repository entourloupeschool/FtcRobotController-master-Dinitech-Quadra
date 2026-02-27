package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.AutoBase;

//@Autonomous(name = "FollowTrajectoryPedroTest - Dinitech", group = "Test")
@Disabled
public class FollowTrajectoryPedroTest extends AutoBase {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        new SequentialCommandGroup(
                new FollowPath(drivePedroSubsystem,
                        new PathChain(
                                new Path(
                                        new BezierLine(
                                                BEGIN_POSE,
                                                BEGIN_POSE.withX(BEGIN_POSE.getX() + 20)
                                        )
                                ),
                                new Path(
                                        new BezierLine(
                                                drivePedroSubsystem.getPose(),
                                                BEGIN_POSE
                                        )
                                )),
                        AUTO_ROBOT_CONSTRAINTS, true),
                new WaitCommand(1000),
                new FollowPath(drivePedroSubsystem,
                        new PathChain(
                                new Path(
                                        new BezierLine(
                                                BEGIN_POSE,
                                                BEGIN_POSE.withX(BEGIN_POSE.getX() + 20)
                                        )
                                ),
                                new Path(
                                        new BezierLine(
                                                drivePedroSubsystem.getPose(),
                                                BEGIN_POSE
                                        )
                                )),
                        AUTO_ROBOT_CONSTRAINTS, true)
        ).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }

}
