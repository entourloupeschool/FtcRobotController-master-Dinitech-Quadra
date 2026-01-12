package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.FlywheelAutoTuner;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * OpMode for automatically tuning the PIDF coefficients of the flywheel
 * shooter.
 * 
 * Controls:
 * - Triangle (Δ): Start auto-tuning
 * - Circle (○): Cancel tuning and restore original values
 * - Cross (X): Stop motor
 * 
 * The auto-tuner will perform velocity step response tests to determine optimal
 * PIDF coefficients for the flywheel shooter system.
 */
@TeleOp(name = "FlywheelAutoTuner - Dinitech", group = "Test")
@Disabled
public class FlywheelAutoTunerOpMode extends DinitechRobotBase {

    private GamepadSubsystem gamepadSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private GamepadWrapper driver;

    private FlywheelAutoTuner autoTuner;
    private boolean tuningActive = false;
    private boolean tuningCompleted = false;
    private FlywheelAutoTuner.TuningProfile selectedProfile = FlywheelAutoTuner.TuningProfile.AGGRESSIVE;

    private PIDFCoefficients tunedPIDF = null;
    private PIDFCoefficients originalPIDF = null;

    @Override
    public void initialize() {
        super.initialize();
        // Initialize subsystems
        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        register(shooterSubsystem);

        driver = gamepadSubsystem.driver;

        // Save original PIDF values
        originalPIDF = shooterSubsystem.getPIDFVelocity();

        // Set up button bindings
        setupButtonBindings();

        telemetry.addLine("=== Flywheel Auto-Tuner ===");
        telemetry.addLine("Profile Selection (D-pad):");
        telemetry.addLine("  ⬆️ UP: BALANCED");
        telemetry.addLine("  ➡️ RIGHT: AGGRESSIVE (fast)");
        telemetry.addLine("  ⬇️ DOWN: CONSERVATIVE (safe)");
        telemetry.addLine("  ⬅️ LEFT: NO_OVERSHOOT (0% overshoot)");
        telemetry.addLine("");
        telemetry.addLine("Press Triangle (Δ) to start auto-tuning");
        telemetry.addLine("Press Circle (○) to cancel and restore");
        telemetry.addLine("Press Cross (X) to stop motor");
        telemetry.addLine("");
        telemetry.addData("Selected Profile", selectedProfile.toString());
        telemetry.addLine("");
        telemetry.addLine("Current PIDF values:");
        telemetry.addData("P", String.format("%.6f", originalPIDF.p));
        telemetry.addData("I", String.format("%.6f", originalPIDF.i));
        telemetry.addData("D", String.format("%.6f", originalPIDF.d));
        telemetry.addData("F", String.format("%.6f", originalPIDF.f));
        telemetry.update();
    }

    private void setupButtonBindings() {
        // D-pad: Select tuning profile (only when not tuning)
        driver.dpad_up.whenPressed(() -> {
            if (!tuningActive) {
                selectedProfile = FlywheelAutoTuner.TuningProfile.BALANCED;
            }
        });

        driver.dpad_right.whenPressed(() -> {
            if (!tuningActive) {
                selectedProfile = FlywheelAutoTuner.TuningProfile.AGGRESSIVE;
            }
        });

        driver.dpad_down.whenPressed(() -> {
            if (!tuningActive) {
                selectedProfile = FlywheelAutoTuner.TuningProfile.CONSERVATIVE;
            }
        });

        driver.dpad_left.whenPressed(() -> {
            if (!tuningActive) {
                selectedProfile = FlywheelAutoTuner.TuningProfile.NO_OVERSHOOT;
            }
        });

        // Triangle: Start auto-tuning
        driver.triangle.whenPressed(() -> {
            if (!tuningActive) {
                startAutoTuning();
            }
        });

        // Circle: Cancel and restore original values
        driver.circle.whenPressed(() -> {
            if (tuningActive && autoTuner != null) {
                autoTuner.cancel();
                tuningActive = false;
                shooterSubsystem.setPIDFVelocity(originalPIDF.p, originalPIDF.i, originalPIDF.d, originalPIDF.f);
                telemetry.addLine("Tuning cancelled - restored original PIDF");
            }
        });

        // Cross: Stop motor
        driver.cross.whenPressed(() -> {
            shooterSubsystem.setVelocity(0);
        });

        // Square: Restore original PIDF (if tuning completed)
        driver.square.whenPressed(() -> {
            if (tuningCompleted && originalPIDF != null) {
                shooterSubsystem.setPIDFVelocity(originalPIDF.p, originalPIDF.i, originalPIDF.d, originalPIDF.f);
                telemetry.addLine("Restored original PIDF values");
            }
        });
    }

    private void startAutoTuning() {
        autoTuner = new FlywheelAutoTuner(shooterSubsystem, telemetry, selectedProfile);
        schedule(autoTuner);
        tuningActive = true;
        tuningCompleted = false;

        telemetry.addLine("Starting auto-tuning...");
        telemetry.addData("Profile", selectedProfile.toString());
        telemetry.addLine("Do not interfere with the robot during tuning!");
    }

    @Override
    public void run() {
        // Check if auto-tuning just completed
        if (tuningActive && autoTuner != null && !autoTuner.isScheduled()) {
            tuningActive = false;
            tuningCompleted = true;
            tunedPIDF = autoTuner.getTunedPIDF();

            telemetry.addLine("=== Auto-Tuning Complete ===");
            telemetry.addLine("");
            telemetry.addLine("Tuned PIDF values (now active):");
            telemetry.addData("P", String.format("%.6f", tunedPIDF.p));
            telemetry.addData("I", String.format("%.6f", tunedPIDF.i));
            telemetry.addData("D", String.format("%.6f", tunedPIDF.d));
            telemetry.addData("F", String.format("%.6f", tunedPIDF.f));
            telemetry.addLine("");
            telemetry.addLine("Original PIDF values:");
            telemetry.addData("Orig P", String.format("%.6f", originalPIDF.p));
            telemetry.addData("Orig I", String.format("%.6f", originalPIDF.i));
            telemetry.addData("Orig D", String.format("%.6f", originalPIDF.d));
            telemetry.addData("Orig F", String.format("%.6f", originalPIDF.f));
            telemetry.addLine("");
            telemetry.addLine("Press Square (□) to restore original values");
            telemetry.addLine("Press Triangle (Δ) to run tuning again");
        }

        // Display status when not tuning
        if (!tuningActive) {
            telemetry.addLine("");
            telemetry.addLine("--- Profile Selection (D-pad) ---");
            telemetry.addLine("⬆️: BALANCED | ➡️: AGGRESSIVE | ⬇️: CONSERVATIVE | ⬅️: NO_OVERSHOOT");
            telemetry.addData("Selected", selectedProfile.toString());
            telemetry.addLine("");
            telemetry.addLine("--- Controls ---");
            telemetry.addLine("△: Start auto-tuning");
            telemetry.addLine("○: Cancel tuning");
            telemetry.addLine("✕: Stop motor");
            if (tuningCompleted) {
                telemetry.addLine("□: Restore original PIDF");
                telemetry.addData("P", String.format("%.6f", tunedPIDF.p));
                telemetry.addData("I", String.format("%.6f", tunedPIDF.i));
                telemetry.addData("D", String.format("%.6f", tunedPIDF.d));
                telemetry.addData("F", String.format("%.6f", tunedPIDF.f));
            }
        }

        super.run();
    }
}
