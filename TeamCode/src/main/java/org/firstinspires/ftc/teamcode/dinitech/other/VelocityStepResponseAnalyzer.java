package org.firstinspires.ftc.teamcode.dinitech.other;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for analyzing step response data from velocity-controlled
 * motors.
 * Used to measure system characteristics like rise time, overshoot, settling
 * time, etc.
 * This data is essential for automatic PIDF tuning of velocity controllers.
 */
public class VelocityStepResponseAnalyzer {

    /**
     * Data point representing velocity at a specific time
     */
    public static class DataPoint {
        public final long timestamp; // milliseconds
        public final double velocity; // encoder ticks per second

        public DataPoint(long timestamp, double velocity) {
            this.timestamp = timestamp;
            this.velocity = velocity;
        }
    }

    /**
     * Results of velocity step response analysis
     */
    public static class Metrics {
        public double riseTime; // Time to reach 90% of target velocity (ms)
        public double overshootPercent; // Percentage overshoot beyond target
        public double settlingTime; // Time to stay within ±2% of target (ms)
        public double steadyStateError; // Final error from target (ticks/sec)
        public double peakTime; // Time to reach peak velocity (ms)
        public double peakVelocity; // Maximum velocity reached (ticks/sec)
        public boolean hasOscillation; // Whether velocity oscillates
        public double oscillationPeriod; // Period of oscillation if present (ms)
        public double targetVelocity; // Target velocity for reference
        public double initialVelocity; // Starting velocity for reference

        @Override
        public String toString() {
            return String.format(
                    "Rise Time: %.0f ms\n" +
                            "Overshoot: %.1f%%\n" +
                            "Settling Time: %.0f ms\n" +
                            "Steady State Error: %.1f ticks/sec\n" +
                            "Peak Time: %.0f ms\n" +
                            "Oscillation: %s%s",
                    riseTime,
                    overshootPercent,
                    settlingTime,
                    steadyStateError,
                    peakTime,
                    hasOscillation ? "Yes" : "No",
                    hasOscillation ? String.format(" (Period: %.0f ms)", oscillationPeriod) : "");
        }
    }

    private final List<DataPoint> dataPoints;
    private double targetVelocity;
    private double initialVelocity;
    private boolean isRecording;

    public VelocityStepResponseAnalyzer() {
        this.dataPoints = new ArrayList<>();
        this.isRecording = false;
    }

    /**
     * Start recording a new velocity step response test
     * 
     * @param initialVel Starting velocity (ticks/sec)
     * @param targetVel  Target velocity to reach (ticks/sec)
     */
    public void startRecording(double initialVel, double targetVel) {
        reset();
        this.initialVelocity = initialVel;
        this.targetVelocity = targetVel;
        this.isRecording = true;
    }

    /**
     * Stop recording data
     */
    public void stopRecording() {
        this.isRecording = false;
    }

    /**
     * Record a data point during the step response
     * 
     * @param velocity  Current velocity in ticks per second
     * @param timestamp Current time in milliseconds
     */
    public void recordSample(double velocity, long timestamp) {
        if (isRecording) {
            dataPoints.add(new DataPoint(timestamp, velocity));
        }
    }

    /**
     * Analyze the recorded velocity step response data
     * 
     * @return Metrics object containing analysis results, or null if insufficient
     *         data
     */
    public Metrics analyze() {
        if (dataPoints.size() < 10) {
            return null; // Not enough data
        }

        Metrics metrics = new Metrics();
        metrics.targetVelocity = targetVelocity;
        metrics.initialVelocity = initialVelocity;

        long startTime = dataPoints.get(0).timestamp;
        double velocityChange = Math.abs(targetVelocity - initialVelocity);

        // Calculate rise time (time to reach 90% of target)
        double ninetyPercentVelocity = initialVelocity
                + 0.9 * velocityChange * Math.signum(targetVelocity - initialVelocity);
        metrics.riseTime = calculateTimeToReach(ninetyPercentVelocity, startTime);

        // Find peak velocity and peak time
        int peakIndex = findPeakIndex();
        metrics.peakVelocity = dataPoints.get(peakIndex).velocity;
        metrics.peakTime = dataPoints.get(peakIndex).timestamp - startTime;

        // Calculate overshoot
        double overshoot = Math.abs(metrics.peakVelocity - targetVelocity);
        metrics.overshootPercent = (velocityChange > 0) ? (100.0 * overshoot / velocityChange) : 0;

        // Calculate settling time (time to stay within ±2% of target)
        double settlingTolerance = Math.max(5.0, 0.02 * Math.abs(targetVelocity));
        metrics.settlingTime = calculateSettlingTime(settlingTolerance, startTime);

        // Calculate steady state error (average of last 10% of samples)
        metrics.steadyStateError = calculateSteadyStateError();

        // Detect oscillations
        detectOscillations(metrics);

        return metrics;
    }

    /**
     * Calculate time to reach a specific velocity
     */
    private double calculateTimeToReach(double velocity, long startTime) {
        boolean isIncreasing = targetVelocity > initialVelocity;

        for (DataPoint point : dataPoints) {
            if (isIncreasing && point.velocity >= velocity) {
                return point.timestamp - startTime;
            } else if (!isIncreasing && point.velocity <= velocity) {
                return point.timestamp - startTime;
            }
        }

        // Never reached - return total time
        return dataPoints.get(dataPoints.size() - 1).timestamp - startTime;
    }

    /**
     * Find the index of the peak (maximum deviation from initial velocity)
     */
    private int findPeakIndex() {
        int peakIndex = 0;
        double maxDeviation = 0;

        for (int i = 0; i < dataPoints.size(); i++) {
            double deviation = Math.abs(dataPoints.get(i).velocity - initialVelocity);
            if (deviation > maxDeviation) {
                maxDeviation = deviation;
                peakIndex = i;
            }
        }

        return peakIndex;
    }

    /**
     * Calculate settling time - time to stay within tolerance of target
     */
    private double calculateSettlingTime(double tolerance, long startTime) {
        // Search backwards from the end to find when it last left the settling band
        for (int i = dataPoints.size() - 1; i >= 0; i--) {
            double error = Math.abs(dataPoints.get(i).velocity - targetVelocity);
            if (error > tolerance) {
                // Found last point outside tolerance
                if (i < dataPoints.size() - 1) {
                    return dataPoints.get(i + 1).timestamp - startTime;
                }
            }
        }

        // Never settled or was always settled
        return dataPoints.get(dataPoints.size() - 1).timestamp - startTime;
    }

    /**
     * Calculate steady state error from last 10% of samples
     */
    private double calculateSteadyStateError() {
        int numSamples = Math.max(5, dataPoints.size() / 10);
        int startIndex = dataPoints.size() - numSamples;

        double sumError = 0;
        for (int i = startIndex; i < dataPoints.size(); i++) {
            sumError += Math.abs(dataPoints.get(i).velocity - targetVelocity);
        }

        return sumError / numSamples;
    }

    /**
     * Detect oscillations by looking for zero crossings of velocity error signal
     */
    private void detectOscillations(Metrics metrics) {
        if (dataPoints.size() < 20) {
            metrics.hasOscillation = false;
            return;
        }

        List<Long> zeroCrossings = new ArrayList<>();

        // Find zero crossings (when error changes sign)
        for (int i = 1; i < dataPoints.size(); i++) {
            double prevError = dataPoints.get(i - 1).velocity - targetVelocity;
            double currError = dataPoints.get(i).velocity - targetVelocity;

            // Check if sign changed (zero crossing)
            if (prevError * currError < 0) {
                zeroCrossings.add(dataPoints.get(i).timestamp);
            }
        }

        // Need at least 2 zero crossings to detect oscillation
        if (zeroCrossings.size() >= 2) {
            metrics.hasOscillation = true;

            // Calculate average period (time between zero crossings * 2)
            double totalPeriod = 0;
            for (int i = 1; i < zeroCrossings.size(); i++) {
                totalPeriod += (zeroCrossings.get(i) - zeroCrossings.get(i - 1)) * 2;
            }
            metrics.oscillationPeriod = totalPeriod / (zeroCrossings.size() - 1);
        } else {
            metrics.hasOscillation = false;
            metrics.oscillationPeriod = 0;
        }
    }

    /**
     * Reset the analyzer for a new test
     */
    public void reset() {
        dataPoints.clear();
        isRecording = false;
    }

    /**
     * Get the current number of recorded samples
     */
    public int getSampleCount() {
        return dataPoints.size();
    }

    /**
     * Check if currently recording
     */
    public boolean isRecording() {
        return isRecording;
    }

    /**
     * Get the recorded data points (for debugging/visualization)
     */
    public List<DataPoint> getDataPoints() {
        return new ArrayList<>(dataPoints); // Return copy to prevent modification
    }
}
