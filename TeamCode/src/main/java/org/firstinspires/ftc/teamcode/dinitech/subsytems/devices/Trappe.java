package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_TELE_INCREMENT;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents the Trappe (door/trapdoor) mechanism of the robot, controlled by a single servo.
 * <p>
 * This class provides high-level methods to open and close the trappe, as well as
 * perform incremental adjustments. It encapsulates the hardware details of the servo
 * and provides a simple interface for controlling the trappe's state.
 *
 * @see SingleServo
 */
public class Trappe extends SingleServo {
    /** State of the door (open/closed) */
    private boolean doorIsOpen = false;

    /**
     * Constructs a new Trappe instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public Trappe(final HardwareMap hardwareMap){
        super(hardwareMap);
        close();
    }


    /**
     * Gets the name of the servo associated with this trappe.
     * @return The name of the trappe servo.
     */
    public String getName() {
        return TRAPPE_SERVO_NAME;
    }

    /**
     * Opens the trappe to a predefined position.
     */
    public void open() {
        rotateToAngle(TRAPPE_OPEN_POSITION);
        doorIsOpen = true;
    }


    /**
     * Closes the trappe to a predefined position.
     */
    public void close() {
        rotateToAngle(TRAPPE_CLOSE_POSITION);
        doorIsOpen = false;
    }

    /**
     * Checks if the trappe is currently open.
     * @return True if the trappe is open, false otherwise.
     */
    public boolean isDoorOpen() {
        return doorIsOpen;
    }

    /**
     * Incrementally rotates the trappe upwards by a predefined amount.
     */
    public void incrementalRotationUp() {
        incrementalRotation(TRAPPE_TELE_INCREMENT);
    }

    /**
     * Incrementally rotates the trappe downwards by a predefined amount.
     */
    public void incrementalRotationDown() {
        incrementalRotation(- TRAPPE_TELE_INCREMENT);
    }


}
