package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TeleDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TestCircles;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
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
    private DriveSubsystem driveSubsystem;
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

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));

        mecanumDrive = new DinitechMecanumDrive(hardwareMap, BEGIN_POSE);
        driveSubsystem = new DriveSubsystem(mecanumDrive, telemetry);
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(new TeleDrive(driveSubsystem, gamepadSubsystem));

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

        driver.bump_right.whenPressed(new ToggleVisionDrive(driveSubsystem, visionSubsystem, gamepadSubsystem));
        driver.bump_left.whenPressed(new TestCircles(driveSubsystem));
    }

}
