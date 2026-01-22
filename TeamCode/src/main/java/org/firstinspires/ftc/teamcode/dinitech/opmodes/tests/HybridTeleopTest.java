package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.transformToPedroCoordinates;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
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

    private DrivePedroSubsystem drivePedroSubsystem;
    private VisionSubsystem visionSubsystem;
    private GamepadSubsystem gamepadSubsystem;

    @Override
    public void initialize() {
        super.initialize();

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE, telemetryM);
        register(drivePedroSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);
        // Register subsystems

        // Set hybrid teleop as default command for drive subsystem
        drivePedroSubsystem.setDefaultCommand(
                new FieldCentricDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));
    }
    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }
}
