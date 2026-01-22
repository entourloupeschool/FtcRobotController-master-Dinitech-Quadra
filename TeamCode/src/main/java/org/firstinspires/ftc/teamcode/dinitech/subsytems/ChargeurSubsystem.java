package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SCALE_CHARGEUR_MOTOR_POWER;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.ChargeurDoubleServo;

/**
 * A command-based subsystem for controlling the Chargeur (loader/intake) mechanism.
 * <p>
 * This subsystem manages the motor responsible for pulling game elements into the robot.
 * It provides simple power control and includes an over-current safety feature to prevent
 * motor damage.
 *
 * @see SubsystemBase
 * @see MotorEx
 */
public class ChargeurSubsystem extends SubsystemBase {
    private final MotorEx motorEx;
    private final ChargeurDoubleServo chargeurDoubleServo;
    public final TelemetryManager telemetryM;

    /**
     * Constructs a new ChargeurSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetryM   The telemetry object for logging.
     */
    public ChargeurSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryM){
        this.motorEx = new MotorEx(hardwareMap, CHARGEUR_MOTOR_NAME);
        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorEx.setRunMode(Motor.RunMode.RawPower);
        
        this.chargeurDoubleServo = new ChargeurDoubleServo(hardwareMap);

        this.telemetryM = telemetryM;
    }



    public void setChargeurPower(double power){
        chargeurDoubleServo.setNormalizedSpeed(3, power);
        setMotorPower(power * SCALE_CHARGEUR_MOTOR_POWER);
    }

    public void incrementChargeurPower(double increment){
        chargeurDoubleServo.incrementNormalizedSpeed(3, increment);
        incrementMotorPower(increment);
    }


    /**
     * Sets the power of the intake motor.
     *
     * @param power The power level, from -1.0 (reverse) to 1.0 (forward).
     */
    public void setMotorPower(double power){
        motorEx.set(power);
    }

    /**
     * Gets the current power of the intake motor.
     *
     * @return The current power level.
     */
    public double getMotorPower(){
        return motorEx.get();
    }

    /**
     * Increments the current power of the intake motor.
     *
     * @param powerIncrement The amount to add to the current power.
     */
    public void incrementMotorPower(double powerIncrement){
        setMotorPower(getMotorPower() + powerIncrement);
    }

    /**
     * Gets the velocity of the intake motor.
     *
     * @return The velocity in ticks per second.
     */
    public double getMotorSpeed(){
        return motorEx.getVelocity();
    }

    /**
     * Checks if the intake motor is currently powered.
     *
     * @return True if the absolute power is greater than a small threshold (0.1).
     */
    public boolean isMotorPowered(){
        return Math.abs(getMotorPower()) > 0.1;
    }

    /**
     * Determines if the motor power should be stopped due to a safety condition.
     *
     * @return True if the motor is in an over-current state.
     */
    public boolean shouldMotorStopPower(){
        return isOverCurrent();
    }

    /**
     * Checks if the motor speed has reached a certain threshold.
     *
     * @param speed The target speed in ticks per second.
     * @return True if the current speed is at or above the target speed.
     */
    public boolean isMotorAtSpeed(double speed){
        return getMotorSpeed() >= speed;
    }

    /**
     * Checks if the motor is currently experiencing an over-current condition.
     *
     * @return True if an over-current condition is detected.
     */
    public boolean isOverCurrent(){
        return motorEx.motorEx.isOverCurrent();
    }

    @Override
    public void periodic(){
        // Automatically stop the motor if an over-current condition is detected.
        if (isOverCurrent()){
            setMotorPower(0);
            telemetryM.addLine("chargeur motor over current");
        }
    }

    private void printChargeurTelemetry(final Telemetry telemetry){
        telemetry.addData("Chargeur Power", "%.2f", getMotorPower());
    }
}
