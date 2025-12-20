package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_SHOOTER_SCALER;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.ChargeurServo;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.SingleContinuousServo;

@TeleOp(name="BasicContinuousServo - Dinitech", group="Test")
public class BasicServo extends DinitechRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;
    private GamepadSubsystem gamepadSubsystem;
    private ChargeurServo chargeurServo;
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        chargeurServo = new ChargeurServo(hardwareMap);

        setupGamePadsButtonBindings();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double velocityIncrement = - m_Driver.getRightY();
        if (Math.abs(velocityIncrement) > 0.05){
            chargeurServo.incrementNormalizedSpeed((velocityIncrement * velocityIncrement * velocityIncrement * velocityIncrement * velocityIncrement)/30);
        }

        telemetry.addData("chargeur normalizedSpeed", chargeurServo.getNormalizedSpeed());

        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver = gamepadSubsystem.driver;
        m_Operator = gamepadSubsystem.operator;

        m_Driver.circle.whileHeld(new RunCommand(() -> chargeurServo.incrementNormalizedSpeed(0.0005)));
        m_Driver.square.whileHeld(new RunCommand(() -> chargeurServo.incrementNormalizedSpeed(-0.0005)));
        m_Driver.triangle.whenPressed(new InstantCommand(() -> chargeurServo.setNormalizedSpeed(0.0)));

    }
}
