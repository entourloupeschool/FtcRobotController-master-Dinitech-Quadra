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
public abstract class SingleContinuousServo {
    /** The FTCLib CRServo instance for continuous rotation servo control. */
    private final CRServo servo;

    /**
     * Constructs a new SingleContinuousServo.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public SingleContinuousServo(final HardwareMap hardwareMap) {
        this.servo = new CRServo(hardwareMap, getName());
    }

    /**
     * Gets the unique hardware name for the servo.
     * This method must be implemented by subclasses to identify the servo in the hardware configuration.
     *
     * @return The hardware name of the servo.
     */
    public abstract String getName();

    /**
     * Gets the current power of the servo.
     *
     * @return The current power, from -1.0 to 1.0.
     */
    public double getNormalizedSpeed() {
        return servo.get();
    }

    /**
     * Sets the power of the servo.
     *
     * @param power The power to set, from -1.0 (full reverse) to 1.0 (full forward).
     */
    public void setNormalizedSpeed(double power) {
        servo.set(power);
    }

    /**
     * Increments the current power of the servo by a given amount.
     *
     * @param increment The amount to add to the current power.
     */
    public void incrementNormalizedSpeed(double increment){
        servo.set(getNormalizedSpeed() + increment);
    }

    /**
     * Checks if the servo's direction is inverted.
     *
     * @return True if the servo is inverted, false otherwise.
     */
    public boolean isInverted(){
        return servo.getInverted();
    }

    /**
     * Sets whether the servo's direction should be inverted.
     *
     * @param inversion True to invert the direction, false for normal direction.
     */
    public void setInverted(boolean inversion){
        servo.setInverted(inversion);
    }

    /**
     * Gets the achievable maximum speed of the servo in ticks per second.
     *
     * @return The maximum speed.
     */
    public double getMaxSpeed(){
        return servo.ACHIEVABLE_MAX_TICKS_PER_SECOND;
    }

    /**
     * Gets the underlying {@link CRServo} instance for advanced control.
     *
     * @return The servo instance.
     */
    public CRServo getServo() {
        return servo;
    }

}
