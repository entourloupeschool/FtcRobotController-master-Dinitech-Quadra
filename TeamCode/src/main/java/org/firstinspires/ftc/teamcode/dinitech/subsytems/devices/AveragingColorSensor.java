package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GAIN_DETECTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_HUE_HIGHER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_RED_RGB_HIGHER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_SATURATION_LOWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PURPLE_HUE_HIGHER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PURPLE_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SAMPLE_SIZE_TEST;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper for a {@link NormalizedColorSensor} that provides smoothed color readings
 * using a moving average filter.
 * <p>
 * This class collects a series of color samples and calculates the average for RGB and HSV
 * values. This helps to reduce noise and provide more stable color detection. It also
 * functions as a {@link DistanceSensor}.
 *
 * @see NormalizedColorSensor
 * @see DistanceSensor
 */
public class AveragingColorSensor {
    public final NormalizedColorSensor sensor;

    // Lists for the moving average
    private final List<Double> redSamples = new ArrayList<>();
    private final List<Double> greenSamples = new ArrayList<>();
    private final List<Double> blueSamples = new ArrayList<>();
//    private List<Double> alphaSamples = new ArrayList<>();
    private final List<Double> hueSamples = new ArrayList<>();
    private final List<Double> saturationSamples = new ArrayList<>();
//    private final List<Double> brightnessSamples = new ArrayList<>();

    /**
     * Constructs a new AveragingColorSensor.
     *
     * @param sensor The {@link NormalizedColorSensor} to wrap.
     */
    public AveragingColorSensor(NormalizedColorSensor sensor) {
        this.sensor = sensor;
        setGain(GAIN_DETECTION);
    }

    /**
     * Reads the latest color data from the sensor and adds it to the sample lists
     * for the moving average.
     */
    public void update() {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Add new values to the lists
        addSampleToList(redSamples, colors.red);
        addSampleToList(greenSamples, colors.green);
        addSampleToList(blueSamples, colors.blue);
//            addSampleToList(alphaSamples, colors.alpha);
        addSampleToList(hueSamples, hsvValues[0]);
        addSampleToList(saturationSamples, hsvValues[1]);
//            addSampleToList(brightnessSamples, hsvValues[2]);

    }

    // Accessor methods for averages
    /**
     * Gets the average red value from the collected samples.
     * @return The average red value.
     */
    public double getAverageRed() { return calculateAverage(redSamples); }
    /**
     * Gets the average green value from the collected samples.
     * @return The average green value.
     */
    public double getAverageGreen() { return calculateAverage(greenSamples); }
    /**
     * Gets the average blue value from the collected samples.
     * @return The average blue value.
     */
    public double getAverageBlue() { return calculateAverage(blueSamples); }
//    public double getAverageAlpha() { return calculateAverage(alphaSamples); }
    /**
     * Gets the average hue value from the collected samples.
     * @return The average hue value.
     */
    public double getAverageHue() { return calculateAverage(hueSamples); }
    /**
     * Gets the average saturation value from the collected samples.
     * @return The average saturation value.
     */
    public double getAverageSaturation() { return calculateAverage(saturationSamples); }
//    public double getAverageBrightness() { return calculateAverage(brightnessSamples); }
    /**
     * Gets the current number of samples collected for averaging.
     * @return The number of samples.
     */
    public int getSampleCount() { return hueSamples.size(); }

    // Color detection methods
    /**
     * Checks if the detected color is green based on the average HSV and RGB values.
     * @return True if the color is green, false otherwise.
     */
    public boolean isGreen() {
        return getAverageRed() > getAverageBlue() && getAverageRed() > getAverageGreen();
//        return isAverageHueBetween(GREEN_HUE_LOWER, GREEN_HUE_HIGHER) &&
//                getAverageGreen() > getAverageBlue() && getAverageSaturation() > GREEN_SATURATION_LOWER && getAverageRed() < GREEN_RED_RGB_HIGHER;
    }

    /**
     * Checks if the detected color is purple based on the average hue value.
     * @return True if the color is purple, false otherwise.
     */
    public boolean isPurple() {
        return getAverageBlue() > getAverageRed() && getAverageBlue() > getAverageGreen();

//        return isAverageHueBetween(PURPLE_HUE_LOWER, PURPLE_HUE_HIGHER);
    }

    /**
     * Gets the distance measured by the sensor.
     * @return The distance in centimeters.
     */
    public double getDistance() {
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
    }

    private boolean isAverageHueBetween(double lowerThreshold, double upperThreshold) {
        double averageHue = getAverageHue();
        return averageHue >= lowerThreshold && averageHue <= upperThreshold;
    }

    /**
     * Adds a sample to the list and maintains the size at SAMPLE_SIZE_TEST.
     */
    private void addSampleToList(List<Double> list, double value) {
        list.add(value);
        if (list.size() > SAMPLE_SIZE_TEST) {
            list.remove(0);
        }
    }

    /**
     * Clears all collected color samples.
     */
    public void clearSamples() {
        redSamples.clear();
        greenSamples.clear();
        blueSamples.clear();
//        alphaSamples.clear();
        hueSamples.clear();
        saturationSamples.clear();
//        brightnessSamples.clear();
    }

    /**
     * Calculates the average of values in a list.
     */
    private double calculateAverage(List<Double> list) {
        if (list.isEmpty()) return 0.0;

        double sum = 0.0;
        for (double value : list) {
            sum += value;
        }
        return sum / list.size();
    }

    /**
     * Gets the current gain of the color sensor.
     * @return The current gain value.
     */
    public double getGain(){
        return sensor.getGain();
    }

    /**
     * Sets the gain of the color sensor.
     * @param gain The new gain value.
     */
    public void setGain(double gain){
        sensor.setGain((float) gain);
    }
}