package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.TrieurAutoTuner;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * TeleOp mode for running the automatic PIDF tuner.
 * 
 * Uses TrieurSubsystem with Moulin motor configured to tune both
 * position P coefficient and velocity PIDF coefficients.
 */
@TeleOp(name = "Trieur Auto-Tuner - Dinitech", group = "Test")
public class TrieurAutoTunerOpMode extends DinitechRobotBase {

    private GamepadWrapper m_Driver;
    private GamepadSubsystem gamepadSubsystem;
    private TrieurSubsystem trieurSubsystem;
    private TrieurAutoTuner autoTuner;

    private boolean tuningStarted = false;
    private boolean tuningComplete = false;
    private PIDFCoefficients tunedValues = null;

    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
        register(trieurSubsystem);

        setupGamePadButtonBindings();

        telemetry.addLine("=== Trieur Auto-Tuner ===");
        telemetry.addLine("Press CROSS (X) to start auto-tuning");
        telemetry.addLine("Press CIRCLE (O) to apply tuned values");
        telemetry.addLine("Press SQUARE to restore original values");
        telemetry.addLine("");

        PIDFCoefficients currentPIDF = trieurSubsystem.getPIDF();
        telemetry.addLine("Current PIDF:");
        telemetry.addData("P", String.format("%.6f", currentPIDF.p));
        telemetry.addData("I", String.format("%.6f", currentPIDF.i));
        telemetry.addData("D", String.format("%.6f", currentPIDF.d));
        telemetry.addData("F", String.format("%.6f", currentPIDF.f));
        telemetry.update();
    }

    @Override
    public void run() {
        // Check if tuning just completed
        if (tuningStarted && !tuningComplete && autoTuner != null) {
            if (!autoTuner.isScheduled()) {
                // Tuning finished
                tuningComplete = true;
                tunedValues = autoTuner.getTunedPIDF();

                telemetry.addLine("");
                telemetry.addLine("=== AUTO-TUNING COMPLETE ===");
                telemetry.addLine("Tuned PIDF values:");
                telemetry.addData("P", String.format("%.6f", tunedValues.p));
                telemetry.addData("I", String.format("%.6f", tunedValues.i));
                telemetry.addData("D", String.format("%.6f", tunedValues.d));
                telemetry.addData("F", String.format("%.6f", tunedValues.f));
                telemetry.addLine("");
                telemetry.addLine("Press CIRCLE (O) to keep these values");
                telemetry.addLine("Press SQUARE to restore original values");
            }
        }

        super.run();
    }

    private void setupGamePadButtonBindings() {
        m_Driver = gamepadSubsystem.driver;

        // Cross button: Start auto-tuning
        m_Driver.cross.whenPressed(new InstantCommand(this::startAutoTuning));

        // Circle button: Apply tuned values
        m_Driver.circle.whenPressed(new InstantCommand(this::applyTunedValues));

        // Square button: Restore original values
        m_Driver.square.whenPressed(new InstantCommand(this::restoreOriginalValues));

        // Triangle button: Show current PIDF
        m_Driver.triangle.whenPressed(new InstantCommand(this::showCurrentPIDF));
    }

    private void startAutoTuning() {
        if (!tuningStarted) {
            tuningStarted = true;
            tuningComplete = false;

            autoTuner = new TrieurAutoTuner(trieurSubsystem, telemetry);
            autoTuner.schedule();

            telemetry.addLine("Auto-tuning started...");
        } else {
            telemetry.addLine("Auto-tuning already in progress or complete!");
        }
    }

    private void applyTunedValues() {
        if (tuningComplete && tunedValues != null) {
            trieurSubsystem.setPIDF(tunedValues.p, tunedValues.i, tunedValues.d, tunedValues.f);

            telemetry.addLine("=== TUNED VALUES APPLIED ===");
            telemetry.addData("P", String.format("%.6f", tunedValues.p));
            telemetry.addData("I", String.format("%.6f", tunedValues.i));
            telemetry.addData("D", String.format("%.6f", tunedValues.d));
            telemetry.addData("F", String.format("%.6f", tunedValues.f));
            telemetry.addLine("");
            telemetry.addLine("Values have been applied to the motor.");
            telemetry.addLine("Note: These will reset when OpMode restarts.");
        } else {
            telemetry.addLine("No tuned values available yet!");
            telemetry.addLine("Run auto-tuning first (press CROSS).");
        }
    }

    private void restoreOriginalValues() {
        if (autoTuner != null) {
            PIDFCoefficients original = autoTuner.getOriginalPIDF();
            trieurSubsystem.setPIDF(original.p, original.i, original.d, original.f);

            telemetry.addLine("=== ORIGINAL VALUES RESTORED ===");
            telemetry.addData("P", String.format("%.6f", original.p));
            telemetry.addData("I", String.format("%.6f", original.i));
            telemetry.addData("D", String.format("%.6f", original.d));
            telemetry.addData("F", String.format("%.6f", original.f));
            telemetry.update();
        } else {
            telemetry.addLine("No original values to restore!");
        }
    }

    private void showCurrentPIDF() {
        PIDFCoefficients current = trieurSubsystem.getPIDF();

        telemetry.addLine("=== CURRENT PIDF VALUES ===");
        telemetry.addData("P", String.format("%.6f", current.p));
        telemetry.addData("I", String.format("%.6f", current.i));
        telemetry.addData("D", String.format("%.6f", current.d));
        telemetry.addData("F", String.format("%.6f", current.f));
    }
}
