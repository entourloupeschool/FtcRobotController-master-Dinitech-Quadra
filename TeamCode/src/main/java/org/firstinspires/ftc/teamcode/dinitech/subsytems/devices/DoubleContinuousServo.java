package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * An abstract base class for controlling a single continuous rotation servo.
 * <p>
 * This class simplifies the use of a continuous servo by providing methods for setting its speed,
 * inverting its direction, and accessing the underlying hardware object. Subclasses are required to
 * provide the specific hardware name of the servo motor they control.
 *
 * @see CRServo
 */
public abstract class DoubleInvertedContinuousServo {
    /** The FTCLib CRServo instance for continuous rotation servo control. */
    private final CRServo servo1;
    private final CRServo servo2;


    /**
     * Constructs a new DoubleInvertedContinuousServo.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public DoubleInvertedContinuousServo(final HardwareMap hardwareMap, String name1, String name2) {
        this.servo1 = new CRServo(hardwareMap, name1);
        this.servo2 = new CRServo(hardwareMap, name2);
    }

    /**
     * Gets the current power of the servo.
     *
     * @return The current power, from -1.0 to 1.0.
     */
    public double getNormalizedSpeed(int crsNumber) {
        switch (crsNumber) {
            case 1:
                return servo1.get();
            case 2:
                return servo2.get();
            default:
                return 0;
        }
    }

    /**
     * Sets the power of the servo.
     *
     * @param crsNumber The crs number to set, from 1 to 3. 3 is both.
     * @param power The power to set, from -1.0 (full reverse) to 1.0 (full forward).
     */
    public void setNormalizedSpeed(int crsNumber, double power) {
        switch (crsNumber) {
            case 1:
                servo1.set(power);
            case 2:
                servo2.set(power);
            case 3:
                servo1.set(power);
                servo2.set(power);
            default:
                return ;
        }
    }

    /**
     * Increments the current power of the servo by a given amount.
     *
     * @para crsNumber the crs number to set, from 1 to 3. 3 is both.
     * @param increment The amount to add to the current power.
     */
    public void incrementNormalizedSpeed(int crsNumber, double increment){
        switch (crsNumber) {
            case 1:
                servo1.set(getNormalizedSpeed(1) + increment);
                break;
            case 2:
                servo2.set(getNormalizedSpeed(2) + increment);
                break;
            case 3:
                servo1.set(getNormalizedSpeed(1) + increment);
                servo2.set(getNormalizedSpeed(2) + increment);
                break;
            default:
                return ;
        }
    }

    /**
     * Checks if the servo's direction is inverted.
     *
     * @param crsNumber The crs number to set, from 1 to 2.
     * @return True if the servo is inverted, false otherwise.
     */
    public boolean isInverted(int crsNumber){
        switch (crsNumber){
            case 1:
                return servo1.getInverted();
            case 2:
                return servo2.getInverted();
            default:
                return false;

        }
    }

    /**
     * Sets whether the servo's direction should be inverted.
     *
     * @param crsNumber The crs number to set, from 1 to 3. 3 is both.
     * @param inversion True to invert the direction, false for normal direction.
     */
    public void setInverted(int crsNumber, boolean inversion){
        switch (crsNumber){
            case 1:
                servo1.setInverted(inversion);
                break;
            case 2:
                servo2.setInverted(inversion);
                break;
            case 3:
                servo1.setInverted(inversion);
                servo2.setInverted(inversion);
                break;
            default:
                return ;
        }
    }

    /**
     * Gets the achievable maximum speed of the servo in ticks per second.
     *
     * @param crsNumber The crs number to set, from 1 to 2.
     * @return The maximum speed.
     */
    public double getMaxSpeed(int crsNumber){
        switch (crsNumber){
            case 1:
                return servo1.ACHIEVABLE_MAX_TICKS_PER_SECOND;
            case 2:
                return servo2.ACHIEVABLE_MAX_TICKS_PER_SECOND;
            default:
                return 0;
        }
    }

    /**
     * Gets the underlying {@link CRServo} instance for advanced control.
     *
     * @param crsNumber The crs number to set, from 1 to 2.
     * @return The servo instance.
     */
    public CRServo getServo(int crsNumber) {
        switch (crsNumber){
            case 1:
                return servo1;
            case 2:
                return servo2;
            default:
                return null;
        }
    }

}
