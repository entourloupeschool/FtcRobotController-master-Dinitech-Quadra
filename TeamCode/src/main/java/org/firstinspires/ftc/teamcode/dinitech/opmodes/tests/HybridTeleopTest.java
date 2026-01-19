package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * Example TeleOp demonstrating the hybrid drive command.
 * 
 * Controls:
 * - Left Stick: Strafe (X/Y translation)
 * - Right Stick X: Rotation
 * - Right Trigger: Speed boost
 * - Left Bumper: Toggle between direct velocity mode (default) and trajectory
 * precision mode
 * 
 * Direct Velocity Mode:
 * - Zero latency response
 * - Standard FTC teleop feel
 * - Best for general driving
 * 
 * Trajectory Precision Mode:
 * - Smooth, controlled motion
 * - Better for precise positioning
 * - ~150ms latency (trajectory duration)
 */
@TeleOp(name = "Hybrid Teleop Test", group = "Test")
public class HybridTeleopTest extends DinitechRobotBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private GamepadSubsystem gamepadSubsystem;

    @Override
    public void initialize() {
        super.initialize();

        driveSubsystem = new DriveSubsystem(hardwareMap, BEGIN_POSE, telemetry);
        register(driveSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        register(visionSubsystem);

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);
        // Register subsystems

        // Set hybrid teleop as default command for drive subsystem
        driveSubsystem.setDefaultCommand(
                new FieldCentricDrive(driveSubsystem, visionSubsystem, gamepadSubsystem));
    }
    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }
}
