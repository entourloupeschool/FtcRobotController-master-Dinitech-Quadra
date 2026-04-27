package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.CLOSE_SHOOT_BLUE_POSE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
@Configurable
public class ShooterSubsystem extends SubsystemBase {
    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final int RUNNING_AVERAGE_SHOOTER_CURRENT_SIZE = 5;
    public static final int CURRENT_SHOOT_OVERFLOW = 1500;
    public static final double MAX_SHOOT_SPEED = 2800; // Ticks per second.
    public static final double SPEED_MARGIN = 15;
    public static double SPEED_MARGIN_SUPER_INTEL =  SPEED_MARGIN * 3;
    public static final double SPEED_INCREMENT_SHOOTER = 10;
    public static final double MAX_RANGE_TO_SHOOT_CM = 345;
    public static final double MIN_RANGE_TO_SHOOT_CM = 97;
    public static final double TELE_SHOOTER_SCALER = 30;

    public static final double P_SHOOTER_VELOCITY_AGGRESSIVE = 80;
    public static final double I_SHOOTER_VELOCITY_AGGRESSIVE = 1.33;
    public static final double D_SHOOTER_VELOCITY_AGGRESSIVE = 0.005;
    public static final double F_SHOOTER_VELOCITY_AGGRESSIVE = 2.1;

    public static double P_SHOOTER_VELOCITY_AGGRESSIVE_3R = P_SHOOTER_VELOCITY_AGGRESSIVE;
    public static double I_SHOOTER_VELOCITY_AGGRESSIVE_3R = 0.0001;
    public static double D_SHOOTER_VELOCITY_AGGRESSIVE_3R = 5.745;
    public static double F_SHOOTER_VELOCITY_AGGRESSIVE_3R = F_SHOOTER_VELOCITY_AGGRESSIVE * 5;
    public static final long SHOOT_REVOLUTION_THEN_WAIT = 300;


    public static double a_Pedro = 5.85; // 5.95;
    public static double b_Pedro = 940; // 1060;

    /**
     * Gives back a linear speed from a range in inches
     * @param rangeInch The range value in inches, positive.
     * @return The speed value, also positive
     */
    public static double linearSpeedFromPedroRange(double rangeInch) {
        return a_Pedro * rangeInch + b_Pedro;
    }
    private enum VelocityPidProfile {
        BASE,
        LOADED
    }

    private final DcMotorEx dcMotorEx;
    private final TelemetryManager telemetryM;
    private final DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;


    public boolean isCurrentOverflow(){
        return getVoltage() > CURRENT_SHOOT_OVERFLOW;
    }

    /**
     * Defines the operational state of the shooter.
     */
    public enum ShooterUsageState {
        TELE,   // Controlled by driver
        VISION, // Controlled by vision auto-aim
        PEDRO,
        STOP,
        NONE    // Not in use
    }

    private double targetSpeed = 0;
    private ShooterUsageState usageState;
    private VelocityPidProfile activeVelocityPidProfile;

    /**
     * Constructs a new ShooterSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetryM   The telemetry object for logging.
     */
    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryM) {
        dcMotorEx = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);

        dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorEx.setMode(runMode);

        applyVelocityPidProfile(VelocityPidProfile.BASE);

        setPower(1);
        setVelocity(0);
        setUsageState(ShooterUsageState.NONE);

        this.telemetryM = telemetryM;
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
     * @param target The target speed.
     * @param margin The allowed tolerance.
     * @return True if the velocity is within the specified range.
     */
    public boolean isSpeedAround(double target, double margin) {
        return Math.abs(getVelocity() - target) <= margin;
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
        return dcMotorEx.getCurrent(CurrentUnit.MILLIAMPS);
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

    private void applyVelocityPidProfile(VelocityPidProfile profile) {
        if (profile == activeVelocityPidProfile) return;

        switch (profile) {
            case BASE:
                setPIDFVelocity(P_SHOOTER_VELOCITY_AGGRESSIVE, I_SHOOTER_VELOCITY_AGGRESSIVE,
                        D_SHOOTER_VELOCITY_AGGRESSIVE, F_SHOOTER_VELOCITY_AGGRESSIVE);
                break;
            case LOADED:
                setPIDFVelocity(P_SHOOTER_VELOCITY_AGGRESSIVE_3R, I_SHOOTER_VELOCITY_AGGRESSIVE_3R,
                        D_SHOOTER_VELOCITY_AGGRESSIVE_3R, F_SHOOTER_VELOCITY_AGGRESSIVE_3R);
                break;
        }

        activeVelocityPidProfile = profile;
    }

    private void updateVelocityPidProfileForLoad() {
        double targetSpeed = getTargetSpeed();
        if (targetSpeed <= 10) {
            applyVelocityPidProfile(VelocityPidProfile.BASE);
            return;
        }

        double speedError = targetSpeed - getVelocity();

        if (activeVelocityPidProfile == VelocityPidProfile.LOADED && speedError > SPEED_MARGIN * 2) {
            applyVelocityPidProfile(VelocityPidProfile.BASE);
        } else if (activeVelocityPidProfile == VelocityPidProfile.BASE && speedError < SPEED_MARGIN * 2) {
            applyVelocityPidProfile(VelocityPidProfile.LOADED);
        }
    }

    /**
     * Gets the current PIDF coefficients for the motor's velocity control.
     * @return The PIDF coefficients.
     */
    public PIDFCoefficients getPIDFVelocity() {
        return dcMotorEx.getPIDFCoefficients(runMode);
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


    @Override
    public void periodic() {
        if (isOverCurrent()) {
            stopMotor();
            telemetryM.addLine("shooter motor over current");
        }

        updateVelocityPidProfileForLoad();

        printShooterTelemetry(telemetryM);
    }

    private void printShooterTelemetry(final TelemetryManager telemetryM) {
//        telemetryM.addData("Shooter Speed (ticks/s)", getVelocity());
//        telemetryM.addData("Target Speed (ticks/s)", getTargetSpeed());
        telemetryM.addData("Shooter State", getUsageState());
//        telemetryM.addData("Shooter PIDF profile", activeVelocityPidProfile);
//        telemetryM.addData("targetReachedSUPERINTEL", isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL));
//        telemetryM.addData("targetSpeedStabilized", isTargetSpeedStabilized());
//        telemetryM.addData("current", getVoltage());

    }
}
