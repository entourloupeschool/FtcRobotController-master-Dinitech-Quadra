package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.ChargeurDoubleServo;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name="BasicContinuousServo - Dinitech", group="Basic")
@Disabled
public class BasicServo extends GornetixRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;
    private GamepadSubsystem gamepadSubsystem;
    private ChargeurDoubleServo chargeurDoubleServo;
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        chargeurDoubleServo = new ChargeurDoubleServo(hardwareMap);

        setupGamePadsButtonBindings();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double velocityIncrement = - m_Driver.getRightY();
        if (Math.abs(velocityIncrement) > 0.05){
            chargeurDoubleServo.incrementNormalizedSpeed(3, (velocityIncrement * velocityIncrement * velocityIncrement * velocityIncrement * velocityIncrement)/30);
        }

        telemetry.addData("chargeur 1 normalizedSpeed", chargeurDoubleServo.getNormalizedSpeed(1));
        telemetry.addData("chargeur 2 normalizedSpeed", chargeurDoubleServo.getNormalizedSpeed(2));


        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver = gamepadSubsystem.getDriver();
        m_Operator = gamepadSubsystem.getOperator();

        m_Driver.circle.whileHeld(new RunCommand(() -> chargeurDoubleServo.incrementNormalizedSpeed(3, 0.0005)));
        m_Driver.square.whileHeld(new RunCommand(() -> chargeurDoubleServo.incrementNormalizedSpeed(3, -0.0005)));
        m_Driver.triangle.whenPressed(new InstantCommand(() -> chargeurDoubleServo.setNormalizedSpeed(3, 0.0)));
        m_Driver.cross.whenPressed(new InstantCommand(() -> chargeurDoubleServo.invertDirection(1)));


    }
}
