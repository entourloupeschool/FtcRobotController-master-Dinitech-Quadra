package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name="TrieurBasic - Dinitech", group="Test")

public class TrieurBasic extends DinitechRobotBase {
    // Gamepads
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;

    private GamepadSubsystem gamepadSubsystem;
    private TrieurSubsystem trieurSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
        register(trieurSubsystem);

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        setupGamePadsButtonBindings();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {




    }
}
