package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * An abstract base class for controlling a single servo motor.
 * <p>
 * This class simplifies the use of a servo by providing methods for rotating to specific angles,
 * performing incremental rotations, and setting angle limits. Subclasses are required to
 * provide the specific hardware name of the servo motor they control.
 *
 * @see ServoEx
 * @see SimpleServo
 */
public abstract class SingleServo {
    /** The FTCLib ServoEx instance for enhanced servo control. */
    private final ServoEx servo;
    /** The minimum and maximum allowed rotation angles for the servo, in degrees. */
    private final double minAngle;
    private final double maxAngle;

    /**
     * Constructs a SingleServo with default angle limits (-135 to 135 degrees).
     *
     * @param hardwareMap The robot's hardware map.
     */
    public SingleServo(final HardwareMap hardwareMap) {
        this(hardwareMap, -135, 135);
    }

    /**
     * Constructs a SingleServo with custom angle limits.
     *
     * @param hardwareMap The robot's hardware map.
     * @param minAngle    The minimum allowed angle in degrees.
     * @param maxAngle    The maximum allowed angle in degrees.
     */
    public SingleServo(final HardwareMap hardwareMap, final double minAngle, final double maxAngle) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.servo = new SimpleServo(hardwareMap, getName(), minAngle, maxAngle, AngleUnit.DEGREES);
    }

    /**
     * Gets the unique hardware name for the servo.
     * This method must be implemented by subclasses to identify the servo in the hardware configuration.
     *
     * @return The hardware name of the servo.
     */
    public abstract String getName();

    /**
     * Gets the current angle of the servo.
     *
     * @return The current angle in degrees.
     */
    public double getAngle() {
        return servo.getAngle(AngleUnit.DEGREES);
    }

    /**
     * Rotates the servo to a specific angle. The angle will be clamped within the defined min/max limits.
     *
     * @param angle The target angle in degrees.
     */
    public void rotateToAngle(final double angle) {
        double clampedAngle = Math.min(Math.max(minAngle, angle), maxAngle);
        servo.turnToAngle(clampedAngle, AngleUnit.DEGREES);
    }

    /**
     * Incrementally rotates the servo by a given amount.
     *
     * @param increment The amount to rotate by, in degrees.
     */
    public void incrementalRotation(final double increment) {
        rotateToAngle(getAngle() + increment);
    }

    /**
     * Moves the servo to its default position (0 degrees).
     */
    public void defaultPosition() {
        servo.turnToAngle(0, AngleUnit.DEGREES);
    }

    /**
     * Gets the underlying {@link ServoEx} instance for advanced control.
     *
     * @return The servo instance.
     */
    public ServoEx getServo() {
        return servo;
    }

    /**
     * Gets the minimum allowed angle for the servo.
     *
     * @return The minimum angle in degrees.
     */
    public double getMinAngle() { return minAngle; }
    /**
     * Gets the maximum allowed angle for the servo.
     *
     * @return The maximum angle in degrees.
     */
    public double getMaxAngle() { return maxAngle; }
}
