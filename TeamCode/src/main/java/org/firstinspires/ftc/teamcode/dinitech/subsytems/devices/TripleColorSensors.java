package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CS1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CS2_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CS3_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

/**
 * Manages a set of three color sensors, providing a unified interface for reading their values.
 * <p>
 * This class abstracts the hardware details of three separate {@link AveragingColorSensor}s
 * and provides methods to check for specific colors (green, purple), measure distances,
 * and manage sensor data updates and sampling.
 *
 * @see AveragingColorSensor
 */
public class TripleColorSensors {

    /** The three color sensors with integrated averaging. */
    public final AveragingColorSensor colorSensor1;
    public final AveragingColorSensor colorSensor2;
    public final AveragingColorSensor colorSensor3;

    /**
     * Constructs a new TripleColorSensors instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public TripleColorSensors(HardwareMap hardwareMap) {
        this.colorSensor1 = new AveragingColorSensor(hardwareMap.get(NormalizedColorSensor.class, CS1_NAME));
        this.colorSensor2 = new AveragingColorSensor(hardwareMap.get(NormalizedColorSensor.class, CS2_NAME));
        this.colorSensor3 = new AveragingColorSensor(hardwareMap.get(NormalizedColorSensor.class, CS3_NAME));
    }

    /**
     * Updates the readings for all three color sensors.
     */
    public void updateAllSensors() {
        colorSensor1.update();
        colorSensor2.update();
//        colorSensor3.update();
    }

    /**
     * Updates the reading for a specific color sensor.
     *
     * @param sensorNumber The number of the sensor to update (1, 2, or 3).
     */
    public void updateSensor(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                colorSensor1.update();
                break;
            case 2:
                colorSensor2.update();
                break;
            case 3:
                colorSensor3.update();
                break;
        }
    }

    /**
     * Clears the collected samples for all three color sensors.
     */
    public void clearSamplesAllSensors() {
        colorSensor1.clearSamples();
        colorSensor2.clearSamples();
        colorSensor3.clearSamples();
    }

    /**
     * Clears the collected samples for a specific color sensor.
     *
     * @param sensorNumber The number of the sensor to clear (1, 2, or 3).
     */
    public void clearSensor(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                colorSensor1.clearSamples();
                break;
            case 2:
                colorSensor2.clearSamples();
                break;
            case 3:
                colorSensor3.clearSamples();
                break;
        }
    }

    /**
     * Checks if a specific sensor detects the color green.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, or 3).
     * @return True if the sensor detects green, false otherwise.
     */
    public boolean isGreen(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.isGreen();
            case 2:
                return colorSensor2.isGreen();
            case 3:
                return colorSensor3.isGreen();
            default:
                return false;
        }
    }

    /**
     * Checks if a specific sensor detects the color purple.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, or 3).
     * @return True if the sensor detects purple, false otherwise.
     */
    public boolean isPurple(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.isPurple();
            case 2:
                return colorSensor2.isPurple();
            case 3:
                return colorSensor3.isPurple();
            default:
                return false;
        }
    }

    /**
     * Checks if a specific sensor detects neither green nor purple.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, or 3).
     * @return True if the sensor detects a color other than green or purple, false otherwise.
     */
    public boolean isNeither(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return !colorSensor1.isGreen() && !colorSensor1.isPurple();
            case 2:
                return !colorSensor2.isGreen() && !colorSensor2.isPurple();
            case 3:
                return !colorSensor3.isGreen() && !colorSensor3.isPurple();
            default:
                return false;
        }
    }

    /**
     * Checks if both sensor 1 and sensor 2 detect green.
     *
     * @return True if both sensors detect green, false otherwise.
     */
    public boolean isBothGreen() {
        return colorSensor1.isGreen() && colorSensor2.isGreen();
    }

    /**
     * Checks if both sensor 1 and sensor 2 detect purple.
     *
     * @return True if both sensors detect purple, false otherwise.
     */
    public boolean isBothPurple() {
        return colorSensor1.isPurple() && colorSensor2.isPurple();
    }

    /**
     * Gets the distance measurement from a specific sensor.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, or 3).
     * @return The distance in centimeters, or 0 if the sensor number is invalid.
     */
    public double getDistance(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getDistance();
            case 2:
                return colorSensor2.getDistance();
            case 3:
                return colorSensor3.getDistance();
            default:
                return 0;
        }
    }

    /**
     * Checks if the distance measured by a specific sensor is less than a given value.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, 3, or 4 for both 1 and 2).
     * @param distanceCM   The distance to compare against, in centimeters.
     * @return True if the measured distance is lower than the specified distance, false otherwise.
     */
    public boolean isDistanceLower(int sensorNumber, double distanceCM) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getDistance() < distanceCM;
            case 2:
                return colorSensor2.getDistance() < distanceCM;
            case 3:
                return colorSensor3.getDistance() < distanceCM;
            case 4:
                return colorSensor1.getDistance() < distanceCM && colorSensor2.getDistance() < distanceCM;
            default:
                return false;
        }
    }

    /**
     * Checks if the distance measured by a specific sensor is within a given range.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, 3, or 4 for both 1 and 2).
     * @param distanceCM   The target distance in centimeters.
     * @param margin       The allowed margin of error in centimeters.
     * @return True if the measured distance is within the specified range, false otherwise.
     */
    public boolean isDistanceBetween(int sensorNumber, double distanceCM, double margin) {
        switch (sensorNumber) {
            case 1:
                double distance1 = colorSensor1.getDistance();
                return distance1 > distanceCM - margin && distance1 < distanceCM + margin;
            case 2:
                double distance2 = colorSensor2.getDistance();
                return distance2 > distanceCM - margin && distance2 < distanceCM + margin;
            case 3:
                double distance3 = colorSensor3.getDistance();
                return distance3 > distanceCM - margin && distance3 < distanceCM + margin;
            case 4:
                double dist1 = colorSensor1.getDistance();
                double dist2 = colorSensor2.getDistance();
                return dist1 > distanceCM - margin && dist1 < distanceCM + margin
                        && dist2 > distanceCM - margin && dist2 < distanceCM + margin;
            default:
                return false;
        }
    }

    /**
     * Gets the number of samples collected by a specific sensor.
     *
     * @param sensorNumber The number of the sensor to check (1, 2, or 3).
     * @return The number of collected samples, or 0 if the sensor number is invalid.
     */
    public int getSampleCount(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getSampleCount();
            case 2:
                return colorSensor2.getSampleCount();
            case 3:
                return colorSensor3.getSampleCount();
            default:
                return 0;
        }
    }

    public double getAverageBlue(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getAverageBlue();
            case 2:
                return colorSensor2.getAverageBlue();
            case 3:
                return colorSensor3.getAverageBlue();
            default:
                return 0;
        }
    }

    public double getAverageRed(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getAverageRed();
            case 2:
                return colorSensor2.getAverageRed();
            case 3:
                return colorSensor3.getAverageRed();
            default:
                return 0;
        }
    }

    public double getAverageGreen(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getAverageGreen();
            case 2:
                return colorSensor2.getAverageGreen();
            case 3:
                return colorSensor3.getAverageGreen();
            default:
                return 0;
        }
    }
    public double getAverageHue(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getAverageHue();
            case 2:
                return colorSensor2.getAverageHue();
            case 3:
                return colorSensor3.getAverageHue();
            default:
                return 0;
        }
    }

    public double getAverageSaturation(int sensorNumber) {
        switch (sensorNumber) {
            case 1:
                return colorSensor1.getAverageSaturation();
            case 2:
                return colorSensor2.getAverageSaturation();
            case 3:
                return colorSensor3.getAverageSaturation();
            default:
                return 0;
        }
    }
}
