package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_SERVO_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents the Chargeur (loader) mechanism, controlled by a continuous rotation servo.
 * <p>
 * This class is a specific implementation of a {@link SingleContinuousServo} and is responsible
 * for providing the hardware name for the loader servo.
 *
 * @see SingleContinuousServo
 */
public class ChargeurServo extends SingleContinuousServo {

    /**
     * Constructs a new ChargeurServo instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public ChargeurServo(final HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * Gets the name of the servo associated with this chargeur.
     * @return The name of the chargeur servo.
     */
    @Override
    public String getName() {
        return CHARGEUR_SERVO_MOTOR_NAME;
    }


}
