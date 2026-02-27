package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_SWITCH_NAME;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;

//@TeleOp(name="BasicMagneticSwitch - Dinitech", group="Basic")
@Disabled

public class BasicMagneticSwitch extends RobotBase {
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
        telemetry.addData("touched :", magneticSwitch.isPressed());
        telemetry.addData("magntic switch value", magneticSwitch.getValue());

        super.run();
    }

}
