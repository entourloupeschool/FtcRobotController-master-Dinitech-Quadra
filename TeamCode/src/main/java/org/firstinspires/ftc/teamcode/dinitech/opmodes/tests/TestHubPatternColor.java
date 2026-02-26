package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "HubPatternColor - Dinitech", group = "Test")
public class TestHubPatternColor extends RobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper m_Driver, m_Operator;
    private HubsSubsystem hubsSubsystem;

    // Create a list of steps for the pattern
    List<Blinker.Step> pattern = new ArrayList<>();

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

        hubsSubsystem = new HubsSubsystem(hardwareMap, true);
        register(hubsSubsystem);

        // Add steps: each Step has a color (ARGB int) and duration in milliseconds
        pattern.add(new Blinker.Step(0xFF00FF00, 500, TimeUnit.MILLISECONDS));  // Green for 500ms
        pattern.add(new Blinker.Step(0xFF0000FF, 500, TimeUnit.MILLISECONDS));  // Blue for 500ms
        pattern.add(new Blinker.Step(0xFFFF0000, 500, TimeUnit.MILLISECONDS));  // Red for 500ms

        hubsSubsystem.setPattern(pattern);
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }

}
