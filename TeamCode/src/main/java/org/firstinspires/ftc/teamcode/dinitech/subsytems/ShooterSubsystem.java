package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.D_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.F_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.I_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.P_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SHOOTER_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_VISION_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kA_SHOOTER_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kS_SHOOTER_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kV_SHOOTER_AGGRESSIVE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;

/**
 * A command-based subsystem for controlling the robot's shooter mechanism.
 * <p>
 * This subsystem manages the motor responsible for launching game elements. It provides
 * precise velocity control using the motor's onboard PIDF controller and supports
 * feedforward control for improved responsiveness. It also includes safety features like
 * over-current protection.
 *
 * @see SubsystemBase
 * @see DcMotorEx
 */
public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx dcMotorEx;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS_SHOOTER_AGGRESSIVE, kV_SHOOTER_AGGRESSIVE, kA_SHOOTER_AGGRESSIVE);
    private final Telemetry telemetry;
    private final DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;

    /**
     * Defines the operational state of the shooter.
     */
    public enum ShooterUsageState {
        TELE,   // Controlled by driver
        VISION, // Controlled by vision auto-aim
        NONE    // Not in use
    }

    private double targetSpeed = 0;
    private double lastTimeStamp, accel, lastVelo;
    private ShooterUsageState usageState;

    /**
     * Constructs a new ShooterSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        dcMotorEx = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);

        dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorEx.setMode(runMode);

        setPIDFVelocity(P_SHOOTER_VELOCITY_AGGRESSIVE, I_SHOOTER_VELOCITY_AGGRESSIVE,
                D_SHOOTER_VELOCITY_AGGRESSIVE,
                F_SHOOTER_VELOCITY_AGGRESSIVE);

        lastTimeStamp = (double) System.nanoTime() / 1E9;

        setVelocity(0);

        this.telemetry = telemetry;
    }

    /**
     * Gets the current position of the shooter motor.
     * @return The position in encoder ticks.
     */
    public int getPosition() {
        return dcMotorEx.getCurrentPosition();
    }

    /**
     * Sets the target velocity of the shooter motor.
     * @param velocity The target velocity in ticks per second.
     */
    public void setVelocity(double velocity) {
        if (velocity >= 0){
            targetSpeed = velocity;
            dcMotorEx.setVelocity(velocity);
        }
    }
//
//    public double getTicksPerRev(){
//        return otherMotor.motor.getMotorType().getTicksPerRev();
//    }
//
//    public double getMaxRPM(){
//        return otherMotor.motor.getMotorType().getMaxRPM();
//    }

    /**
     * Gets the current velocity of the shooter motor.
     * @return The current velocity in ticks per second.
     */
    public double getVelocity() {
        return dcMotorEx.getVelocity();
    }

    /**
     * Sets the raw power of the shooter motor.
     * @param power The power level, from -1.0 to 1.0.
     */
    public void setPower(double power) {
        dcMotorEx.setPower(power);
    }

    /**
     * Gets the current power of the shooter motor.
     * @return The current power level.
     */
    public double getPower() {
        return dcMotorEx.getPower();
    }

    /**
     * Checks if the shooter motor is currently powered.
     * @return True if the power is greater than 0.
     */
    public boolean isPowered() {
        return getPower() > 0;
    }

    /**
     * Increments the target velocity by a given amount.
     * @param velocityIncrement The amount to increment the velocity by.
     */
    public void incrementVelocity(double velocityIncrement) {
        setVelocity(getTargetSpeed() + velocityIncrement);
    }

    /**
     * Increments the power by a given amount.
     * @param powerIncrement The amount to increment the power by.
     */
    public void incrementPower(double powerIncrement) {
        double newPower = getPower() + powerIncrement;
        if (newPower <= 1 && newPower >= 0) {
            setPower(newPower);
        }
    }

    /**
     * Checks if the current velocity is at or above a specified speed.
     * @param speed The speed to compare against.
     * @return True if the velocity is greater than or equal to the specified speed.
     */
    public boolean isSpeedSuperiorOrEqual(double speed) {
        return getVelocity() >= speed;
    }

    /**
     * Checks if the current velocity is exactly a specified speed.
     * @param speed The speed to compare against.
     * @return True if the velocity is equal to the specified speed.
     */
    public boolean isSpeedEqual(double speed) {
        return getVelocity() == speed;
    }

    /**
     * Checks if the current velocity is within a given margin of a specified speed.
     * @param speed The target speed.
     * @param margin The allowed tolerance.
     * @return True if the velocity is within the specified range.
     */
    public boolean isSpeedAround(double speed, double margin) {
        return Math.abs(getVelocity() - speed) <= margin;
    }


    public boolean isTargetSpeedStabilized() {
        return Math.abs(getRawAcceleration()) == 0 && isSpeedAround(getTargetSpeed(), SPEED_MARGIN_VISION_SHOOT);
    }

    /**
     * Checks if the current velocity is within a given margin of the target speed.
     * @param margin The allowed tolerance.
     * @return True if the velocity is within the specified range of the target.
     */
    public boolean isAroundTargetSpeed(double margin) {
        return isSpeedAround(getTargetSpeed(), margin);
    }

    /**
     * Checks if the motor is over-current.
     * @return True if an over-current condition is detected.
     */
    public boolean isOverCurrent() {
        return dcMotorEx.isOverCurrent();
    }

    /**
     * Gets the current drawn by the motor.
     * @return The current in Amperes.
     */
    public double getVoltage() {
        return dcMotorEx.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isBusy() {
        return dcMotorEx.isBusy();
    }

    /**
     * Sets the PIDF coefficients for the motor's velocity control.
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     * @param f The feedforward coefficient.
     */
    public void setPIDFVelocity(double p, double i, double d, double f) {
        dcMotorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    /**
     * Gets the current PIDF coefficients for the motor's velocity control.
     * @return The PIDF coefficients.
     */
    public PIDFCoefficients getPIDFVelocity() {
        return dcMotorEx.getPIDFCoefficients(runMode);
    }

    /**
     * Sets the feedforward coefficients for the motor.
     * @param kS The static friction feedforward.
     * @param kV The velocity feedforward.
     * @param kA The acceleration feedforward.
     */
    public void setFF(double kS, double kV, double kA) {
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    /**
     * Gets the current feedforward coefficients.
     * @return An array containing the kS, kV, and kA coefficients.
     */
    public double[] getFF() {
        return new double[] { feedforward.ks, feedforward.kv, feedforward.ka };
    }

    /**
     * Stops the shooter motor by setting its power to zero.
     */
    public void stopMotor() {
        setVelocity(0);
    }

    /**
     * Gets the current target speed.
     * @return The target speed in ticks per second.
     */
    public double getTargetSpeed() {
        return targetSpeed;
    }

    /**
     * Sets the current usage state of the shooter.
     * @param state The new ShooterUsageState.
     */
    public void setUsageState(ShooterUsageState state) {
        this.usageState = state;
    }

    /**
     * Gets the current usage state of the shooter.
     * @return The current ShooterUsageState.
     */
    public ShooterUsageState getUsageState() {
        return usageState;
    }

    /**
     * Calculates the raw acceleration of the shooter motor.
     * @return The acceleration in ticks per second squared.
     */
    public double getRawAcceleration(){
        double velo = getVelocity();
        if (velo != lastVelo) {
            double currentTime = (double) System.nanoTime() / 1E9;
            double dt = currentTime - lastTimeStamp;
            accel = (velo - lastVelo) / dt;
            lastVelo = velo;
            lastTimeStamp = currentTime;
            return accel;
        }
        return 0;
    }


    @Override
    public void periodic() {
        if (isOverCurrent()) {
            stopMotor();
            telemetry.addLine("shooter motor over current");
        }

        printShooterTelemetry(telemetry);
    }

    private void printShooterTelemetry(final Telemetry telemetry) {
        telemetry.addData("Shooter Speed (ticks/s)", "%.2f", getVelocity());
        telemetry.addData("Target Speed (ticks/s)", "%.2f", getTargetSpeed());
        telemetry.addData("Shooter State", getUsageState());
    }
}
