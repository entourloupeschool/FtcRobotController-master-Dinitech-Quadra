package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.transformToPedroCoordinates;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TestCircles;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/*
Very basic usage of mecanum drive
 */
@TeleOp(name="BasicVisionMecanumDrive - Dinitech", group="Test")
public class BasicVisionMecanumDrive extends DinitechRobotBase {

    // Gamepads
    private DrivePedroSubsystem drivePedroSubsystem;
    private DinitechMecanumDrive mecanumDrive;
    private VisionSubsystem visionSubsystem;
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper driver;
    private GamepadWrapper operator;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE, telemetryM);
        register(drivePedroSubsystem);
        drivePedroSubsystem.setDefaultCommand(new RobotCentricDrive(drivePedroSubsystem, gamepadSubsystem));

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
        driver = gamepadSubsystem.driver;
        operator = gamepadSubsystem.operator;

        driver.bump_right.whenPressed(new ToggleVisionDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));
    }

}
