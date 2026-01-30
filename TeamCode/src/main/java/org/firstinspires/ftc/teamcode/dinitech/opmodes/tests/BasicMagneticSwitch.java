package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CS1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_SWITCH_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SHOOTER_MOTOR_NAME;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;

@TeleOp(name="BasicMagneticSwitch - Dinitech", group="Basic")

public class BasicMagneticSwitch extends DinitechRobotBase {
    public TouchSensor magneticSwitch;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        magneticSwitch = hardwareMap.get(TouchSensor.class, MAGNETIC_SWITCH_NAME);
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();

        telemetry.addData("touched :", magneticSwitch.isPressed());
        telemetry.addData("magntic switch value", magneticSwitch.getValue());
    }

}
