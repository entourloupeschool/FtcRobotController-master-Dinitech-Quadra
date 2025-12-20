package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_SWITCH_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Represents a magnetic switch, which functions as a digital touch sensor.
 * <p>
 * This class provides a simple interface for interacting with a magnetic switch,
 * allowing the robot to detect the presence of a magnetic field.
 *
 * @see TouchSensor
 */
public class MagneticSwitch {
    /** The underlying touch sensor that represents the magnetic switch. */
    public final TouchSensor magneticSwitch;

    /**
     * Constructs a new MagneticSwitch instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public MagneticSwitch(HardwareMap hardwareMap){
        this.magneticSwitch = hardwareMap.get(TouchSensor.class, MAGNETIC_SWITCH_NAME);
    }

    /**
     * Checks if the magnetic switch is currently activated (detecting a magnet).
     *
     * @return True if a magnet is detected, false otherwise.
     */
    public boolean isMagnetic(){
        return magneticSwitch.isPressed();
    }

    /**
     * Gets the raw value from the magnetic switch.
     * For a digital sensor, this typically returns 0.0 or 1.0.
     *
     * @return The raw sensor value.
     */
    public double getMagneticValue(){
        return magneticSwitch.getValue();
    }
}
