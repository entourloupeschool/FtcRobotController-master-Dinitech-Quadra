package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.other.VelocityStepResponseAnalyzer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

/**
 * Command that automatically tunes PIDF coefficients for the flywheel shooter
 * motor.
 * 
 * Uses velocity-based step response analysis to determine optimal PID gains and
 * feedforward coefficient. The flywheel is a velocity-controlled system that
 * spins
 * at a target angular rate (ticks/second).
 */
public class FlywheelAutoTuner extends CommandBase {

    /**
     * Tuning profile for different response characteristics
     */
    public enum TuningProfile {
        NO_OVERSHOOT, // Zero overshoot, slowest response
        CONSERVATIVE, // Minimal overshoot, slower response
        BALANCED, // Balance between speed and stability
        AGGRESSIVE // Fast response, minimal overshoot tolerance (DEFAULT)
    }

    /**
     * Tuning phases
     */
    private enum TuningPhase {
        INITIALIZING, // Spin up to baseline velocity
        STEP_RESPONSE_TEST, // Perform velocity step tests
        CALCULATING, // Calculate optimal PIDF
        VALIDATION, // Validate tuned values
        COMPLETE // Tuning finished
    }

    private final ShooterSubsystem shooterSubsystem;
    private final Telemetry telemetry;
    private final VelocityStepResponseAnalyzer analyzer;
    private final TuningProfile profile;

    private TuningPhase currentPhase;
    private int currentStepTest;

    // Step response variables
    private double stepTestStartVelocity;
    private double stepTestTargetVelocity;
    private long stepTestStartTime;

    // Tuned PIDF values
    private double tunedP;
    private double tunedI;
    private double tunedD;
    private double tunedF;

    // Original PIDF values (for restoration if needed)
    private PIDFCoefficients originalPIDF;

    // Test parameters
    private static final int NUM_STEP_TESTS = 6; // Number of step tests to perform
    private static final long STEP_TEST_DURATION_MS = 5000; // Duration of each step test
    private static final long SETTLING_TIME_MS = 1000; // Time to wait between tests
    private static final int VALIDATION_TESTS = 3;

    private int validationCount;
    private long settlingStartTime;
    private boolean isSettling;

    // Accumulated metrics from multiple tests
    private double totalOvershoot;
    private double totalRiseTime;
    private double totalSettlingTime;
    private int oscillationCount;

    // Test velocity targets (in ticks/second) - expanded range
    private static final double[] TEST_VELOCITIES = { 400, 800, 1200, 1600, 2000, 2400 };

    // Voltage accumulator for F calculation
    private double totalVoltage;
    private int voltageReadings;

    /**
     * Create a new flywheel auto-tuner command with AGGRESSIVE profile
     * 
     * @param shooterSubsystem The shooter subsystem to tune
     * @param telemetry        Telemetry for displaying progress
     */
    public FlywheelAutoTuner(ShooterSubsystem shooterSubsystem, Telemetry telemetry) {
        this(shooterSubsystem, telemetry, TuningProfile.AGGRESSIVE);
    }

    /**
     * Create a new flywheel auto-tuner command with specified profile
     * 
     * @param shooterSubsystem The shooter subsystem to tune
     * @param telemetry        Telemetry for displaying progress
     * @param profile          Tuning profile to use
     */
    public FlywheelAutoTuner(ShooterSubsystem shooterSubsystem, Telemetry telemetry, TuningProfile profile) {
        this.shooterSubsystem = shooterSubsystem;
        this.telemetry = telemetry;
        this.analyzer = new VelocityStepResponseAnalyzer();
        this.profile = profile;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        currentPhase = TuningPhase.INITIALIZING;
        currentStepTest = 0;
        validationCount = 0;
        isSettling = false;

        // Reset accumulated metrics
        totalOvershoot = 0;
        totalRiseTime = 0;
        totalSettlingTime = 0;
        oscillationCount = 0;
        totalVoltage = 0;
        voltageReadings = 0;

        // Save original PIDF values
        originalPIDF = shooterSubsystem.getPIDFVelocity();

        // Start with reasonable initial PIDF values for testing
        // These should allow the motor to spin up but may not be optimal
        shooterSubsystem.setPIDFVelocity(originalPIDF.p, originalPIDF.i, originalPIDF.d, originalPIDF.f);

        telemetry.addLine("=== Flywheel Auto-Tuning Started ===");
        telemetry.addLine("Using velocity-based step response method");
        telemetry.addData("Tuning Profile", profile.toString());

        // Spin up to initial test velocity
        shooterSubsystem.setVelocity(0);
    }

    @Override
    public void execute() {
        switch (currentPhase) {
            case INITIALIZING:
                executeInitializing();
                break;
            case STEP_RESPONSE_TEST:
                executeStepResponseTest();
                break;
            case CALCULATING:
                executeCalculating();
                break;
            case VALIDATION:
                executeValidation();
                break;
            case COMPLETE:
                // Do nothing, waiting for command to finish
                break;
        }

        updateTelemetry();
    }

    /**
     * Phase 1: Initialize and prepare for testing
     */
    private void executeInitializing() {
        // Wait a moment for motor to settle at zero
        if (System.currentTimeMillis() - stepTestStartTime > 500 || stepTestStartTime == 0) {
            if (stepTestStartTime == 0) {
                stepTestStartTime = System.currentTimeMillis();
            } else {
                // Start first step response test
                startNextStepTest();
                currentPhase = TuningPhase.STEP_RESPONSE_TEST;
            }
        }
    }

    /**
     * Phase 2: Perform velocity step response tests
     */
    private void executeStepResponseTest() {
        if (isSettling) {
            // Stop motor during settling period
            shooterSubsystem.setVelocity(0);

            // Wait for system to settle before next test
            if (System.currentTimeMillis() - settlingStartTime > SETTLING_TIME_MS) {
                isSettling = false;

                if (currentStepTest < NUM_STEP_TESTS) {
                    // Start next test
                    startNextStepTest();
                } else {
                    // All tests complete, move to calculation
                    currentPhase = TuningPhase.CALCULATING;
                }
            }
            return;
        }

        long currentTime = System.currentTimeMillis();
        double currentVelocity = shooterSubsystem.getVelocity();

        // Record data point
        analyzer.recordSample(currentVelocity, currentTime);

        // Accumulate voltage readings for F calculation
        totalVoltage += Math.abs(shooterSubsystem.getVoltage());
        voltageReadings++;

        // Check if test duration elapsed
        if (currentTime - stepTestStartTime > STEP_TEST_DURATION_MS) {
            analyzer.stopRecording();

            // Analyze this test
            VelocityStepResponseAnalyzer.Metrics metrics = analyzer.analyze();
            if (metrics != null) {
                totalOvershoot += metrics.overshootPercent;
                totalRiseTime += metrics.riseTime;
                totalSettlingTime += metrics.settlingTime;
                if (metrics.hasOscillation) {
                    oscillationCount++;
                }
            }

            // Start settling period
            isSettling = true;
            settlingStartTime = currentTime;
        }
    }

    /**
     * Start the next velocity step response test
     */
    private void startNextStepTest() {
        stepTestStartVelocity = (currentStepTest == 0) ? 0 : TEST_VELOCITIES[currentStepTest - 1];
        stepTestTargetVelocity = TEST_VELOCITIES[currentStepTest];

        // Set target velocity
        shooterSubsystem.setVelocity(stepTestTargetVelocity);

        analyzer.startRecording(stepTestStartVelocity, stepTestTargetVelocity);
        stepTestStartTime = System.currentTimeMillis();

        currentStepTest++;
    }

    /**
     * Phase 3: Calculate optimal PIDF based on collected data
     */
    private void executeCalculating() {
        // Calculate average metrics
        double avgOvershoot = totalOvershoot / NUM_STEP_TESTS;
        double avgRiseTime = totalRiseTime / NUM_STEP_TESTS;
        double avgSettlingTime = totalSettlingTime / NUM_STEP_TESTS;
        boolean hasOscillation = oscillationCount > (NUM_STEP_TESTS / 2);

        // Calculate average voltage during tests
        double avgVoltage = (voltageReadings > 0) ? totalVoltage / voltageReadings : 0;

        // Calculate PIDF using velocity-specific tuning rules
        calculatePIDFFromMetrics(avgOvershoot, avgRiseTime, avgSettlingTime, hasOscillation, avgVoltage);

        // Apply tuned PIDF
        shooterSubsystem.setPIDFVelocity(tunedP, tunedI, tunedD, tunedF);

        // Move to validation phase
        currentPhase = TuningPhase.VALIDATION;
        validationCount = 0;

        // Start first validation test
        shooterSubsystem.setVelocity(TEST_VELOCITIES[0]);
    }

    /**
     * Calculate PIDF values from step response metrics
     * Uses empirical tuning rules optimized for velocity-controlled flywheel
     * systems, adjusted based on selected tuning profile
     */
    private void calculatePIDFFromMetrics(double overshoot, double riseTime, double settlingTime,
            boolean hasOscillation, double avgVoltage) {
        // === CALCULATE F (FEEDFORWARD) FIRST ===
        // F is the primary coefficient for velocity control
        // F = Voltage / TargetVelocity (in appropriate units)
        // Estimate F from average test data
        double avgTestVelocity = 0;
        for (double vel : TEST_VELOCITIES) {
            avgTestVelocity += vel;
        }
        avgTestVelocity /= TEST_VELOCITIES.length;

        if (avgVoltage > 0 && avgTestVelocity > 0) {
            tunedF = avgVoltage / avgTestVelocity;
        } else {
            // Fallback to original F if calculation failed
            tunedF = originalPIDF.f;
        }

        // === TUNE P BASED ON RISE TIME AND OVERSHOOT ===
        // Profile-specific baseline P values
        double baselineP;
        double targetRiseTimeLow, targetRiseTimeHigh;
        double aggressiveMultiplier;

        switch (profile) {
            case AGGRESSIVE:
                baselineP = 2.0; // Higher baseline for faster response
                targetRiseTimeLow = 500; // Target 500-1000ms rise time
                targetRiseTimeHigh = 1000;
                aggressiveMultiplier = 1.5; // More aggressive adjustments
                break;
            case BALANCED:
                baselineP = 1.5; // Moderate baseline
                targetRiseTimeLow = 800; // Target 800-1500ms rise time
                targetRiseTimeHigh = 1500;
                aggressiveMultiplier = 1.2;
                break;
            case CONSERVATIVE:
                baselineP = 1.0; // Conservative baseline
                targetRiseTimeLow = 1200; // Target 1200-2000ms rise time
                targetRiseTimeHigh = 2000;
                aggressiveMultiplier = 1.1;
                break;
            case NO_OVERSHOOT:
            default:
                baselineP = 0.5; // Very low baseline for zero overshoot
                targetRiseTimeLow = 2000; // Target 2000-3000ms rise time (slow)
                targetRiseTimeHigh = 3000;
                aggressiveMultiplier = 1.0; // No aggressive adjustments
                break;
        }

        tunedP = baselineP;

        // Adjust P based on rise time (profile-dependent)
        if (riseTime > targetRiseTimeHigh * 1.5) {
            // Very slow response - increase P significantly
            tunedP = baselineP * 2.5 * aggressiveMultiplier;
        } else if (riseTime > targetRiseTimeHigh) {
            // Slow response - increase P
            tunedP = baselineP * 1.8 * aggressiveMultiplier;
        } else if (riseTime > targetRiseTimeHigh * 0.8) {
            // Slightly slow - increase P moderately
            tunedP = baselineP * 1.3;
        } else if (riseTime < targetRiseTimeLow * 0.5) {
            // Very fast - reduce P
            tunedP = baselineP * 0.6;
        } else if (riseTime < targetRiseTimeLow) {
            // A bit fast - slightly reduce
            tunedP = baselineP * 0.8;
        }
        // Within target range: use baseline P

        // Adjust P based on overshoot (stricter for NO_OVERSHOOT and CONSERVATIVE, more
        // lenient for AGGRESSIVE)
        double overshootThresholdHigh, overshootThresholdMed, overshootThresholdLow;

        if (profile == TuningProfile.NO_OVERSHOOT) {
            // NO_OVERSHOOT: Extremely strict - any overshoot is unacceptable
            overshootThresholdHigh = 5; // >5% is severe
            overshootThresholdMed = 2; // >2% is moderate
            overshootThresholdLow = 0.5; // >0.5% reduce further
        } else {
            overshootThresholdHigh = (profile == TuningProfile.AGGRESSIVE) ? 30
                    : (profile == TuningProfile.BALANCED) ? 25 : 20;
            overshootThresholdMed = (profile == TuningProfile.AGGRESSIVE) ? 20
                    : (profile == TuningProfile.BALANCED) ? 15 : 10;
            overshootThresholdLow = (profile == TuningProfile.AGGRESSIVE) ? 10
                    : (profile == TuningProfile.BALANCED) ? 5 : 3;
        }

        if (overshoot > overshootThresholdHigh) {
            // Severe overshoot - reduce P significantly
            tunedP *= (profile == TuningProfile.NO_OVERSHOOT) ? 0.3 : 0.5;
        } else if (overshoot > overshootThresholdMed) {
            // Moderate overshoot - reduce P
            tunedP *= (profile == TuningProfile.NO_OVERSHOOT) ? 0.5 : 0.7;
        } else if (overshoot < overshootThresholdLow && riseTime < targetRiseTimeHigh) {
            // Very little overshoot and reasonably fast - can be more aggressive
            // (but not for NO_OVERSHOOT profile)
            if (profile != TuningProfile.NO_OVERSHOOT) {
                tunedP *= aggressiveMultiplier;
            }
        }

        // === CALCULATE I (INTEGRAL) ===
        // I helps eliminate steady-state velocity error
        // For flywheels, moderate I is useful to maintain constant velocity under load
        if (settlingTime > 2500) {
            // Very slow settling - moderate I
            tunedI = tunedP * 0.15;
        } else if (settlingTime > 1800) {
            // Slow settling - small I
            tunedI = tunedP * 0.10;
        } else if (settlingTime > 1200) {
            // Acceptable settling - minimal I
            tunedI = tunedP * 0.05;
        } else {
            // Good settling - very minimal I
            tunedI = tunedP * 0.02;
        }

        // === CALCULATE D (DERIVATIVE) ===
        // D helps reduce overshoot and dampen oscillations in velocity
        // For NO_OVERSHOOT profile, use significant D to prevent any overshoot
        if (profile == TuningProfile.NO_OVERSHOOT) {
            // Always use strong damping for zero overshoot
            tunedD = tunedP * 0.25;
        } else if (hasOscillation) {
            // System is oscillating - need damping
            tunedD = tunedP * 0.15;
        } else if (overshoot > 20) {
            // High overshoot - add damping
            tunedD = tunedP * 0.10;
        } else if (overshoot > 10) {
            // Moderate overshoot - light damping
            tunedD = tunedP * 0.05;
        } else {
            // Minimal overshoot - minimal or no damping
            tunedD = 0.0;
        }

        // === SAFETY LIMITS ===
        // Ensure values are within safe operational ranges
        tunedP = Math.max(0.1, Math.min(tunedP, 10.0)); // P: 0.1 - 10.0
        tunedI = Math.max(0.0, Math.min(tunedI, 2.0)); // I: 0.0 - 2.0
        tunedD = Math.max(0.0, Math.min(tunedD, 1.0)); // D: 0.0 - 1.0
        tunedF = Math.max(0.0, Math.min(tunedF, 0.1)); // F: 0.0 - 0.1
    }

    /**
     * Phase 4: Validate tuned values with test velocities
     */
    private void executeValidation() {
        // Check if velocity has settled at current target
        double targetVel = TEST_VELOCITIES[validationCount % TEST_VELOCITIES.length];
        double currentVel = shooterSubsystem.getVelocity();
        double error = Math.abs(currentVel - targetVel);
        double tolerance = Math.max(20, 0.05 * targetVel); // 5% tolerance or 20 ticks/sec

        if (error <= tolerance) {
            validationCount++;

            if (validationCount < VALIDATION_TESTS) {
                // Set next validation velocity
                shooterSubsystem.setVelocity(TEST_VELOCITIES[validationCount % TEST_VELOCITIES.length]);
            } else {
                // Validation complete - stop motor
                shooterSubsystem.setVelocity(0);
                currentPhase = TuningPhase.COMPLETE;
            }
        }
    }

    /**
     * Update telemetry with current tuning status
     */
    private void updateTelemetry() {
        telemetry.addLine("=== Flywheel Auto-Tuning Progress ===");
        telemetry.addData("Profile", profile.toString());
        telemetry.addData("Phase", currentPhase.toString());

        // Always show motor status
        telemetry.addLine("--- Motor Status ---");
        telemetry.addData("Current Velocity", String.format("%.1f ticks/sec", shooterSubsystem.getVelocity()));
        telemetry.addData("Target Velocity", String.format("%.1f ticks/sec", shooterSubsystem.getTargetSpeed()));
        telemetry.addData("Voltage", String.format("%.2f A", shooterSubsystem.getVoltage()));

        switch (currentPhase) {
            case INITIALIZING:
                telemetry.addLine("Preparing for tests...");
                break;
            case STEP_RESPONSE_TEST:
                if (isSettling) {
                    telemetry.addLine("Settling (motor stopping)...");
                    long settleRemaining = SETTLING_TIME_MS - (System.currentTimeMillis() - settlingStartTime);
                    telemetry.addData("Settle Time", String.format("%.1fs", settleRemaining / 1000.0));
                } else {
                    telemetry.addData("Step Test", currentStepTest + "/" + NUM_STEP_TESTS);
                    telemetry.addData("Samples", analyzer.getSampleCount());
                    long elapsed = System.currentTimeMillis() - stepTestStartTime;
                    telemetry.addData("Progress", String.format("%.1f%%", 100.0 * elapsed / STEP_TEST_DURATION_MS));
                    telemetry.addData("Target", String.format("%.0f ticks/sec", stepTestTargetVelocity));
                }
                break;
            case CALCULATING:
                telemetry.addLine("Analyzing data...");
                if (NUM_STEP_TESTS > 0) {
                    telemetry.addData("Avg Overshoot", String.format("%.1f%%", totalOvershoot / NUM_STEP_TESTS));
                    telemetry.addData("Avg Rise Time", String.format("%.0f ms", totalRiseTime / NUM_STEP_TESTS));
                }
                break;
            case VALIDATION:
                telemetry.addData("Validation", validationCount + "/" + VALIDATION_TESTS);
                telemetry.addData("Test Velocity",
                        String.format("%.0f ticks/sec", TEST_VELOCITIES[validationCount % TEST_VELOCITIES.length]));
                break;
            case COMPLETE:
                telemetry.addLine("=== Tuning Complete ===");
                telemetry.addData("P", String.format("%.6f", tunedP));
                telemetry.addData("I", String.format("%.6f", tunedI));
                telemetry.addData("D", String.format("%.6f", tunedD));
                telemetry.addData("F", String.format("%.6f", tunedF));
                telemetry.addLine("");
                telemetry.addLine("Original values:");
                telemetry.addData("Orig P", String.format("%.6f", originalPIDF.p));
                telemetry.addData("Orig I", String.format("%.6f", originalPIDF.i));
                telemetry.addData("Orig D", String.format("%.6f", originalPIDF.d));
                telemetry.addData("Orig F", String.format("%.6f", originalPIDF.f));
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == TuningPhase.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the motor
        shooterSubsystem.setVelocity(0);

        if (interrupted) {
            // Restore original PIDF if interrupted
            shooterSubsystem.setPIDFVelocity(originalPIDF.p, originalPIDF.i, originalPIDF.d, originalPIDF.f);
            telemetry.addLine("Auto-tuning interrupted - restored original PIDF");
        } else {
            telemetry.addLine("Auto-tuning complete!");
            telemetry.addLine("New PIDF values applied.");
        }
    }

    /**
     * Get the tuned PIDF coefficients
     */
    public PIDFCoefficients getTunedPIDF() {
        return new PIDFCoefficients(tunedP, tunedI, tunedD, tunedF);
    }

    /**
     * Get the original PIDF coefficients
     */
    public PIDFCoefficients getOriginalPIDF() {
        return originalPIDF;
    }
}
