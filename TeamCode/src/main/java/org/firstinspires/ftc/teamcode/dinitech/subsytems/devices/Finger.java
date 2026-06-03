package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents the Finger mechanism of the robot, controlled by a single servo.
 * @see SingleServo
 */
@Configurable
public class Finger extends SingleServo {
    public static final String FINGER_SERVO_NAME = "doigt";
    public static double FINGER_OPEN_POSITION = -10;
    public static double FINGER_CLOSE_POSITION = 10;
    public static long FINGER_OPEN_TIME = 625;
    public static long FINGER_CLOSE_TIME = FINGER_OPEN_TIME+50;
    public static double FINGER_TELE_INCREMENT = 1;

    /** State of the finger (open/closed) */
    private boolean fingerIsOpen = false;
    private void setFingerIsOpen(boolean newFingerIsOpen){
        fingerIsOpen = newFingerIsOpen;
    }
    public boolean getFingerIsOpen(){
        return fingerIsOpen;
    }



    /**
     * Constructs a new Trappe instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public Finger(final HardwareMap hardwareMap){
        super(hardwareMap);
    }


    /**
     * Gets the name of the servo associated with this trappe.
     * @return The name of the trappe servo.
     */
    public String getName() {
        return FINGER_SERVO_NAME;
    }

    /**
     * Opens the trappe to a predefined position.
     */
    public void open() {
        rotateToAngle(FINGER_OPEN_POSITION);
        setFingerIsOpen(true);
    }


    /**
     * Closes the trappe to a predefined position.
     */
    public void close() {
        rotateToAngle(FINGER_CLOSE_POSITION);
        setFingerIsOpen(false);
    }

    public void toggleFinger(){
        if(getFingerIsOpen()) close();
        else open();
    }
}
