package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_SERVO_DROITE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_SERVO_GAUCHE_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents the Chargeur (loader) mechanism, controlled by a continuous rotation servo.
 * <p>
 * This class is a specific implementation of a {@link DoubleContinuousServo} and is responsible
 * for providing the hardware name for the loader servo.
 *
 * @see DoubleContinuousServo
 */
public class ChargeurDoubleServo extends DoubleContinuousServo {

    /**
     * Constructs a new ChargeurServo instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public ChargeurDoubleServo(final HardwareMap hardwareMap) {
        super(hardwareMap, CHARGEUR_SERVO_GAUCHE_MOTOR_NAME, CHARGEUR_SERVO_DROITE_MOTOR_NAME);
        this.getServo(2).setInverted(true);
    }
}
