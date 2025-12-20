package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name="TestsGampads - Dinitech", group="Test")
public class TestGamepads extends DinitechRobotBase {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();
        GamepadSubsystem gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
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
