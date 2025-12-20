package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_NAME;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public final Telemetry telemetry;

    /**
     * Constructs a new ChargeurSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public ChargeurSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.motorEx = new MotorEx(hardwareMap, CHARGEUR_MOTOR_NAME);
        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorEx.setRunMode(Motor.RunMode.RawPower);

        this.telemetry = telemetry;
    }

    /**
     * Sets the power of the intake motor.
     *
     * @param power The power level, from -1.0 (reverse) to 1.0 (forward).
     */
    public void setPower(double power){
        motorEx.set(power);
    }

    /**
     * Gets the current power of the intake motor.
     *
     * @return The current power level.
     */
    public double getPower(){
        return motorEx.get();
    }

    /**
     * Increments the current power of the intake motor.
     *
     * @param powerIncrement The amount to add to the current power.
     */
    public void incrementPower(double powerIncrement){
        setPower(getPower() + powerIncrement);
    }

    /**
     * Gets the velocity of the intake motor.
     *
     * @return The velocity in ticks per second.
     */
    public double getSpeed(){
        return motorEx.getVelocity();
    }

    /**
     * Checks if the intake motor is currently powered.
     *
     * @return True if the absolute power is greater than a small threshold (0.1).
     */
    public boolean isPowered(){
        return Math.abs(getPower()) > 0.1;
    }

    /**
     * Determines if the motor power should be stopped due to a safety condition.
     *
     * @return True if the motor is in an over-current state.
     */
    public boolean shouldStopPower(){
        return isOverCurrent();
    }

    /**
     * Checks if the motor speed has reached a certain threshold.
     *
     * @param speed The target speed in ticks per second.
     * @return True if the current speed is at or above the target speed.
     */
    public boolean isAtSpeed(double speed){
        return getSpeed() >= speed;
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
            setPower(0);
        }
    }

    private void printChargeurTelemetry(final Telemetry telemetry){
        telemetry.addData("Chargeur Power", "%.2f", getPower());
    }
}
