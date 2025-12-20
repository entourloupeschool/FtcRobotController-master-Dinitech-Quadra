package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.other.StepResponseAnalyzer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;

/**
 * A command that automatically tunes the PIDF coefficients for the trieur's moulin motor.
 * <p>
 * This command implements a state machine to guide the tuning process. It uses a
 * unidirectional step response analysis, which is critical for systems with mechanical
 * slack, such as a chain drive. The moulin is only tested in the forward direction to ensure
 * consistent and reliable data.
 * <p>
 * The tuning process consists of several phases:
 * <ol>
 *     <li><b>INITIALIZING:</b> Moves the moulin to a known starting position.</li>
 *     <li><b>STEP_RESPONSE_TEST:</b> Performs a series of forward movements (step inputs) and records
 *     the system's response using a {@link StepResponseAnalyzer}.</li>
 *     <li><b>CALCULATING:</b> Averages the metrics from the tests (e.g., overshoot, rise time) and
 *     uses empirical tuning rules to calculate new P, I, and D coefficients.</li>
 *     <li><b>VALIDATION:</b> Applies the new coefficients and performs a series of test movements to
 *     ensure the system is stable and performs well.</li>
 *     <li><b>COMPLETE:</b> The process is finished, and the new coefficients are active.</li>
 * </ol>
 */
public class TrieurAutoTuner extends CommandBase {

    private enum TuningPhase {
        INITIALIZING, STEP_RESPONSE_TEST, CALCULATING, VALIDATION, COMPLETE
    }

    private final TrieurSubsystem trieurSubsystem;
    private final Telemetry telemetry;
    private final StepResponseAnalyzer analyzer;

    private TuningPhase currentPhase;
    private int currentStepTest;

    private int stepTestTargetPosition;
    private long stepTestStartTime;

    private double tunedP, tunedI, tunedD, tunedF;
    private PIDFCoefficients originalPIDF;

    private static final int NUM_STEP_TESTS = 3;
    private static final long STEP_TEST_DURATION_MS = 2500;
    private static final long SETTLING_TIME_MS = 500;
    private static final int VALIDATION_MOVEMENTS = 3;
    private int validationCount;
    private long settlingStartTime;
    private boolean isSettling;

    private double totalOvershoot, totalRiseTime, totalSettlingTime;
    private int oscillationCount;

    // Initial PIDF values for the test
    private double pInit = 0.3, iInit = 0.0005, dInit = 0.05, fInit = 0.00000044;

    /**
     * Creates a new TrieurAutoTuner command.
     *
     * @param trieurSubsystem The trieur subsystem to be tuned.
     * @param telemetry       The telemetry object for displaying progress.
     */
    public TrieurAutoTuner(TrieurSubsystem trieurSubsystem, Telemetry telemetry) {
        this.trieurSubsystem = trieurSubsystem;
        this.telemetry = telemetry;
        this.analyzer = new StepResponseAnalyzer();
        addRequirements(trieurSubsystem);
    }

    @Override
    public void initialize() {
        currentPhase = TuningPhase.INITIALIZING;
        currentStepTest = 0;
        validationCount = 0;
        isSettling = false;
        totalOvershoot = 0; totalRiseTime = 0; totalSettlingTime = 0; oscillationCount = 0;

        originalPIDF = trieurSubsystem.getPIDF();
        trieurSubsystem.setPIDF(pInit, iInit, dInit, fInit);

        telemetry.addLine("=== Auto-Tuning Started ===");
        trieurSubsystem.rotateToMoulinPosition(1, true);
    }

    @Override
    public void execute() {
        switch (currentPhase) {
            case INITIALIZING: executeInitializing(); break;
            case STEP_RESPONSE_TEST: executeStepResponseTest(); break;
            case CALCULATING: executeCalculating(); break;
            case VALIDATION: executeValidation(); break;
            case COMPLETE: break; // Do nothing, wait for isFinished()
        }
        updateTelemetry();
    }

    private void executeInitializing() {
        if (!trieurSubsystem.shouldMoulinStopPower()) trieurSubsystem.setMoulinPower(1.0);
        if (Math.abs(trieurSubsystem.getMoulinMotorRemainingDistance()) < 10) {
            startNextStepTest();
            currentPhase = TuningPhase.STEP_RESPONSE_TEST;
        }
    }

    private void executeStepResponseTest() {
        if (isSettling) {
            trieurSubsystem.setMoulinPower(0);
            if (System.currentTimeMillis() - settlingStartTime > SETTLING_TIME_MS) {
                isSettling = false;
                if (currentStepTest < NUM_STEP_TESTS) {
                    startNextStepTest();
                } else {
                    currentPhase = TuningPhase.CALCULATING;
                }
            }
            return;
        }

        long currentTime = System.currentTimeMillis();
        analyzer.recordSample(trieurSubsystem.getMoulinMotorPosition(), currentTime);

        if (currentTime - stepTestStartTime > STEP_TEST_DURATION_MS) {
            analyzer.stopRecording();
            StepResponseAnalyzer.Metrics metrics = analyzer.analyze();
            if (metrics != null) {
                totalOvershoot += metrics.overshootPercent;
                totalRiseTime += metrics.riseTime;
                totalSettlingTime += metrics.settlingTime;
                if (metrics.hasOscillation) oscillationCount++;
            }
            isSettling = true;
            settlingStartTime = currentTime;
        }
    }

    private void startNextStepTest() {
        currentStepTest++;
        int startPosition = trieurSubsystem.getMoulinMotorPosition();
        stepTestTargetPosition = startPosition + INTERVALLE_TICKS_MOULIN;
        trieurSubsystem.setMoulinTargetPosition(stepTestTargetPosition);
        analyzer.startRecording(startPosition, stepTestTargetPosition);
        stepTestStartTime = System.currentTimeMillis();
    }

    private void executeCalculating() {
        double avgOvershoot = totalOvershoot / NUM_STEP_TESTS;
        double avgRiseTime = totalRiseTime / NUM_STEP_TESTS;
        double avgSettlingTime = totalSettlingTime / NUM_STEP_TESTS;
        boolean hasOscillation = oscillationCount > (NUM_STEP_TESTS / 2);

        calculatePIDFFromMetrics(avgOvershoot, avgRiseTime, avgSettlingTime, hasOscillation);

        trieurSubsystem.setPIDF(tunedP, tunedI, tunedD, tunedF);
        currentPhase = TuningPhase.VALIDATION;
        validationCount = 0;
        trieurSubsystem.rotateToMoulinPosition(3, false);
    }

    private void calculatePIDFFromMetrics(double overshoot, double riseTime, double settlingTime, boolean hasOscillation) {
        tunedP = Math.max(pInit, 5.0);
        if (riseTime > 1000) tunedP *= 2.0;
        else if (riseTime > 700) tunedP *= 1.5;
        else if (riseTime < 200) tunedP *= 0.7;
        else if (riseTime < 350) tunedP *= 0.85;

        if (overshoot > 25) tunedP *= 0.6;
        else if (overshoot > 15) tunedP *= 0.75;
        else if (overshoot < 2 && riseTime < 600) tunedP *= 1.15;

        if (hasOscillation) tunedD = tunedP * 0.25;
        else if (overshoot > 20) tunedD = tunedP * 0.20;
        else if (overshoot > 10) tunedD = tunedP * 0.12;
        else if (overshoot > 5) tunedD = tunedP * 0.08;
        else tunedD = tunedP * 0.05;

        if (settlingTime > 2000) tunedI = tunedP * 0.08;
        else if (settlingTime > 1500) tunedI = tunedP * 0.04;
        else if (settlingTime > 1000) tunedI = tunedP * 0.02;
        else tunedI = tunedP * 0.01;
        tunedI *= 0.5; // Reduce I for chain slack

        tunedF = 0.0; // F is not used for RUN_TO_POSITION

        // Safety limits
        tunedP = Math.max(1.0, Math.min(tunedP, 20.0));
        tunedI = Math.max(0.0, Math.min(tunedI, 3.0));
        tunedD = Math.max(0.0, Math.min(tunedD, 5.0));
    }

    private void executeValidation() {
        if (Math.abs(trieurSubsystem.getMoulinMotorRemainingDistance()) < 10) {
            validationCount++;
            if (validationCount >= VALIDATION_MOVEMENTS) {
                currentPhase = TuningPhase.COMPLETE;
            } else {
                int currentPos = trieurSubsystem.getMoulinPosition();
                trieurSubsystem.rotateToMoulinPosition((currentPos % 6) + 1, false);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Phase", currentPhase);
        telemetry.addData("Position", trieurSubsystem.getMoulinMotorPosition());
        telemetry.addData("Target", trieurSubsystem.getMoulinMotorTargetPosition());
        if (currentPhase == TuningPhase.COMPLETE) {
            telemetry.addLine("=== Tuning Complete ===");
            telemetry.addData("P", "%.6f", tunedP);
            telemetry.addData("I", "%.6f", tunedI);
            telemetry.addData("D", "%.6f", tunedD);
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == TuningPhase.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            trieurSubsystem.setPIDF(originalPIDF.p, originalPIDF.i, originalPIDF.d, originalPIDF.f);
            telemetry.addLine("Auto-tuning interrupted, PIDF restored.");
        } else {
            telemetry.addLine("Auto-tuning complete. New PIDF values applied.");
        }
    }

    public PIDFCoefficients getTunedPIDF() {
        return new PIDFCoefficients(tunedP, tunedI, tunedD, tunedF);
    }
}
