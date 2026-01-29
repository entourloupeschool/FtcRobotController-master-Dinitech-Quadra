package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

@Autonomous(name = "FollowTrajectoryPedroTest - Dinitech", group = "Test")
public class FollowTrajectoryPedroTest extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private VisionSubsystem visionSubsystem;
    private DrivePedroSubsystem drivePedroSubsystem;


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE, telemetryM);
        register(drivePedroSubsystem);

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
