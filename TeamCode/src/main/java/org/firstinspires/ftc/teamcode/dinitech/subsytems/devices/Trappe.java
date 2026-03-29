package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.TRAPPE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.TRAPPE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.TRAPPE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.TRAPPE_TELE_INCREMENT;

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
    private boolean trappeIsOpen = false;
    private void setTrappeIsOpen(boolean newTrappeIsOpen){
        trappeIsOpen = newTrappeIsOpen;
    }
    public boolean getTrappeIsOpen(){
        return trappeIsOpen;
    }



    /**
     * Constructs a new Trappe instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public Trappe(final HardwareMap hardwareMap){
        super(hardwareMap);
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
        setTrappeIsOpen(true);
    }


    /**
     * Closes the trappe to a predefined position.
     */
    public void close() {
        rotateToAngle(TRAPPE_CLOSE_POSITION);
        setTrappeIsOpen(false);
    }

    public void toggleTrappe(){
        if(getTrappeIsOpen()) close();
        else open();
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
