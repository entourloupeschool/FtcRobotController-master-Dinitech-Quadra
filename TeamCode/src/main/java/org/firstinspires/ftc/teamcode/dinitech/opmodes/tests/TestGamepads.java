package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;

//@TeleOp(name="TestsGampads - Dinitech", group="Test")
@Disabled

public class TestGamepads extends RobotBase {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();
        GamepadSubsystem gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }
}
