package org.firstinspires.ftc.teamcode.dinitech.other;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for analyzing step response data from motor movements.
 * Used to measure system characteristics like rise time, overshoot, settling
 * time, etc.
 * This data is essential for automatic PIDF tuning.
 */
public class StepResponseAnalyzer {

    /**
     * Data point representing position at a specific time
     */
    public static class DataPoint {
        public final long timestamp; // milliseconds
        public final int position; // encoder ticks

        public DataPoint(long timestamp, int position) {
            this.timestamp = timestamp;
            this.position = position;
        }
    }

    /**
     * Results of step response analysis
     */
    public static class Metrics {
        public double riseTime; // Time to reach 90% of target (ms)
        public double overshootPercent; // Percentage overshoot beyond target
        public double settlingTime; // Time to stay within ±2% of target (ms)
        public double steadyStateError; // Final error from target (ticks)
        public double peakTime; // Time to reach peak value (ms)
        public int peakValue; // Maximum position reached (ticks)
        public boolean hasOscillation; // Whether system oscillates
        public double oscillationPeriod; // Period of oscillation if present (ms)
        public int targetPosition; // Target position for reference
        public int initialPosition; // Starting position for reference

        @Override
        public String toString() {
            return String.format(
                    "Rise Time: %.0f ms\n" +
                            "Overshoot: %.1f%%\n" +
                            "Settling Time: %.0f ms\n" +
                            "Steady State Error: %.0f ticks\n" +
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
    private int targetPosition;
    private int initialPosition;
    private boolean isRecording;

    public StepResponseAnalyzer() {
        this.dataPoints = new ArrayList<>();
        this.isRecording = false;
    }

    /**
     * Start recording a new step response test
     * 
     * @param initialPos Starting position
     * @param targetPos  Target position to reach
     */
    public void startRecording(int initialPos, int targetPos) {
        reset();
        this.initialPosition = initialPos;
        this.targetPosition = targetPos;
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
     * @param position  Current position in encoder ticks
     * @param timestamp Current time in milliseconds
     */
    public void recordSample(int position, long timestamp) {
        if (isRecording) {
            dataPoints.add(new DataPoint(timestamp, position));
        }
    }

    /**
     * Analyze the recorded step response data
     * 
     * @return Metrics object containing analysis results, or null if insufficient
     *         data
     */
    public Metrics analyze() {
        if (dataPoints.size() < 10) {
            return null; // Not enough data
        }

        Metrics metrics = new Metrics();
        metrics.targetPosition = targetPosition;
        metrics.initialPosition = initialPosition;

        long startTime = dataPoints.get(0).timestamp;
        int travelDistance = Math.abs(targetPosition - initialPosition);

        // Calculate rise time (time to reach 90% of target)
        int ninetyPercentPosition = initialPosition
                + (int) (0.9 * travelDistance * Math.signum(targetPosition - initialPosition));
        metrics.riseTime = calculateTimeToReach(ninetyPercentPosition, startTime);

        // Find peak value and peak time
        int peakIndex = findPeakIndex();
        metrics.peakValue = dataPoints.get(peakIndex).position;
        metrics.peakTime = dataPoints.get(peakIndex).timestamp - startTime;

        // Calculate overshoot
        int overshoot = Math.abs(metrics.peakValue - targetPosition);
        metrics.overshootPercent = (travelDistance > 0) ? (100.0 * overshoot / travelDistance) : 0;

        // Calculate settling time (time to stay within ±2% of target)
        int settlingTolerance = Math.max(1, (int) (0.02 * travelDistance));
        metrics.settlingTime = calculateSettlingTime(settlingTolerance, startTime);

        // Calculate steady state error (average of last 10% of samples)
        metrics.steadyStateError = calculateSteadyStateError();

        // Detect oscillations
        detectOscillations(metrics);

        return metrics;
    }

    /**
     * Calculate time to reach a specific position
     */
    private double calculateTimeToReach(int position, long startTime) {
        boolean isIncreasing = targetPosition > initialPosition;

        for (DataPoint point : dataPoints) {
            if (isIncreasing && point.position >= position) {
                return point.timestamp - startTime;
            } else if (!isIncreasing && point.position <= position) {
                return point.timestamp - startTime;
            }
        }

        // Never reached - return total time
        return dataPoints.get(dataPoints.size() - 1).timestamp - startTime;
    }

    /**
     * Find the index of the peak (maximum deviation from initial position)
     */
    private int findPeakIndex() {
        int peakIndex = 0;
        int maxDeviation = 0;

        for (int i = 0; i < dataPoints.size(); i++) {
            int deviation = Math.abs(dataPoints.get(i).position - initialPosition);
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
    private double calculateSettlingTime(int tolerance, long startTime) {
        // Search backwards from the end to find when it last left the settling band
        for (int i = dataPoints.size() - 1; i >= 0; i--) {
            int error = Math.abs(dataPoints.get(i).position - targetPosition);
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
            sumError += Math.abs(dataPoints.get(i).position - targetPosition);
        }

        return sumError / numSamples;
    }

    /**
     * Detect oscillations by looking for zero crossings of error signal
     */
    private void detectOscillations(Metrics metrics) {
        if (dataPoints.size() < 20) {
            metrics.hasOscillation = false;
            return;
        }

        List<Long> zeroCrossings = new ArrayList<>();

        // Find zero crossings (when error changes sign)
        for (int i = 1; i < dataPoints.size(); i++) {
            int prevError = dataPoints.get(i - 1).position - targetPosition;
            int currError = dataPoints.get(i).position - targetPosition;

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
