package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "HubPatternColor - Dinitech", group = "Test")
public class TestHubPatternColor extends GornetixRobotBase {
    private GamepadSubsystem gamepadSubsystem;

    private GamepadWrapper m_Driver, m_Operator;
    private HubsSubsystem hubsSubsystem;


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        m_Driver = gamepadSubsystem.getDriver();
        m_Operator = gamepadSubsystem.getOperator();

        hubsSubsystem = new HubsSubsystem(hardwareMap);
        register(hubsSubsystem);


    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double rightTrigger = m_Driver.getRightTriggerValue();

        if (rightTrigger > 0.01) {
            hubsSubsystem.setLedColor(Math.round((float) rightTrigger * 255));
            hubsSubsystem.setPattern();
        }
        super.run();
    }

}
