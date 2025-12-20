package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.D_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.I_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.P_SHOOTER_VELOCITY_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SHOOTER_MOTOR_NAME;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * An alternative implementation of the shooter subsystem that uses FTCLib's {@link MotorEx} for velocity control.
 * <p>
 * This subsystem provides similar functionality to {@link ShooterSubsystem} but leverages the
 * advanced features of {@link MotorEx}, such as built-in velocity PID control and feedforward gains.
 * This can lead to more precise and responsive shooter performance.
 *
 * @see ShooterSubsystem
 * @see MotorEx
 */
public class ShooterVelocitySubsystem extends SubsystemBase {
    private final MotorEx motorEx;
    private final Telemetry telemetry;
    private final Motor.RunMode runMode = Motor.RunMode.VelocityControl;

    private double targetSpeed = 0;
    private boolean visionShooting;

    /**
     * Constructs a new ShooterVelocitySubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public ShooterVelocitySubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motorEx = new MotorEx(hardwareMap, SHOOTER_MOTOR_NAME);
        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorEx.setRunMode(runMode);
        setPID(P_SHOOTER_VELOCITY_AGGRESSIVE, I_SHOOTER_VELOCITY_AGGRESSIVE,
                D_SHOOTER_VELOCITY_AGGRESSIVE);

        setVisionShooting(false);

        this.telemetry = telemetry;
    }

    // Methods for controlling and querying the state of the shooter

    public int getPosition() {
        return motorEx.getCurrentPosition();
    }

    public void setVelocity(double velocity) {
        if (velocity >= 0) {
            motorEx.setVelocity(velocity);
            targetSpeed = velocity;
        }
    }

    public double getVelocity() {
        return motorEx.getVelocity();
    }

    public double getAcceleration(){
        return motorEx.getAcceleration();
    }

    public void setPower(double power) {
        if (power >= 0) {
            motorEx.set(power);
        }
    }

    public double getPower() {
        return motorEx.get();
    }

    public boolean isPowered() {
        return getPower() > 0;
    }

    public void incrementVelocity(double velocityIncrement) {
        double newVelocity = getTargetSpeed() + velocityIncrement;
        if (newVelocity <= MAX_SHOOT_SPEED && newVelocity >= 0) {
            setVelocity(newVelocity);
        }
    }

    public void incrementPower(double powerIncrement) {
        double newPower = getPower() + powerIncrement;
        if (newPower <= 1 && newPower >= 0) {
            setPower(newPower);
        }
    }

    public boolean isSpeedSuperiorOrEqual(double speed) {
        return getVelocity() >= speed;
    }

    public boolean isSpeedEqual(double speed) {
        return getVelocity() == speed;
    }

    public boolean isSpeedAround(double speed, double margin) {
        return Math.abs(getVelocity() - speed) <= margin;
    }

    public boolean isAtTargetSpeed() {
        return isSpeedEqual(getTargetSpeed());
    }

    public boolean isAroundTargetSpeed(double margin) {
        return isSpeedAround(getTargetSpeed(), margin);
    }

    public boolean isOverCurrent() {
        return motorEx.motorEx.isOverCurrent();
    }

    public double getVoltage() {
        return motorEx.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isBusy() {
        return motorEx.motorEx.isBusy();
    }

    public void setPID(double p, double i, double d) {
        motorEx.setVeloCoefficients(p, i, d);
    }

    public PIDCoefficients getPID() {
        double[] pidVelo = motorEx.getVeloCoefficients();
        return new PIDCoefficients(pidVelo[0], pidVelo[1], pidVelo[2]);
    }

    public void setFF(double kS, double kV, double kA) {
        motorEx.setFeedforwardCoefficients(kS, kV, kA);
    }

    public double[] getFF() {
        return motorEx.getFeedforwardCoefficients();
    }

    public void stopMotor() {
        motorEx.set(0);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setVisionShooting(boolean b) {
        visionShooting = b;
    }

    public boolean isVisionShooting() {
        return visionShooting;
    }

    @Override
    public void periodic() {
        if (isOverCurrent()) {
            setPower(0);
        }
        printShooterTelemetry(telemetry);
    }

    private void printShooterTelemetry(final Telemetry telemetry) {
        telemetry.addData("Shooter Speed (ticks/s)", "%.2f", getVelocity());
        telemetry.addData("Target Speed", "%.2f", getTargetSpeed());
    }
}
