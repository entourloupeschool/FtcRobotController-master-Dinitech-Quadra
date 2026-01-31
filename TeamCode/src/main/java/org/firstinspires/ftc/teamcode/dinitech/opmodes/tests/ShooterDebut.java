package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "ShooterDebut - Dinitech", group = "Test")

public class ShooterDebut extends GornetixRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;

    private GamepadSubsystem gamepadSubsystem;
    private ShooterSubsystem shooterSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
        register(shooterSubsystem);
        shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));

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
     * Initialize GamepadEx wrappers for driver and operator.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver = gamepadSubsystem.driver;
        m_Operator = gamepadSubsystem.operator;
    }
}
