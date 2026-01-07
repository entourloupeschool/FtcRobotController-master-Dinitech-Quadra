package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

//@TeleOp(name="BasicColorCamera - Dinitech", group="Test")
@Disabled
public class BasicColorDetectionCamera extends DinitechRobotBase {
    private VisionSubsystem visionSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        // Add processors as needed (or don't add them)
        visionSubsystem.addColorProcessor(10);
        register(visionSubsystem);
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }
}
