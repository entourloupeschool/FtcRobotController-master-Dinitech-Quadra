package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static com.arcrobotics.ftclib.hardware.motors.Motor.RunMode.PositionControl;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.D_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.I_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.P_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kA_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kS_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.kV_MOULIN_AGGRESSIVE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * An alternative implementation of the Moulin mechanism that uses velocity control for positioning.
 * <p>
 * This class leverages the FTCLib {@link MotorEx} to control the Moulin motor. Unlike the
 * standard {@link Moulin} class which uses {@code RUN_TO_POSITION}, this implementation uses
 * {@code PositionControl} mode, which is a velocity-based position controller. This approach allows
 * for more granular control over the motor's behavior and enables the use of FTCLib's built-in
 * PID and feedforward tuning capabilities.
 * <p>
 * The external interface for position control remains consistent with the original {@link Moulin} class,
 * providing the same methods for moving between the 6 discrete positions.
 *
 * @see Moulin
 * @see MotorEx
 */
public class MoulinVelocityControlled {
    private final MotorEx motorEx;
    public static final int MIN_POSITION = 1;
    public static final int MAX_POSITION = 6;
    public static final int TOTAL_POSITIONS = 6;

    private static int moulinPosition;

    /**
     * Constructs a new MoulinVelocityControlled instance.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public MoulinVelocityControlled(final HardwareMap hardwareMap) {
        motorEx = new MotorEx(hardwareMap, MOULIN_MOTOR_NAME);
        configureMoulin();
    }

    /**
     * Configures the Moulin motor with the necessary settings for velocity-controlled positioning.
     */
    public void configureMoulin() {
        motorEx.stopAndResetEncoder();
        motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        setTargetMotorPositionTolerance(MOULIN_POSITION_TOLERANCE);
        setFF(kS_MOULIN_AGGRESSIVE, kV_MOULIN_AGGRESSIVE, kA_MOULIN_AGGRESSIVE);
        setPID(P_MOULIN_AGGRESSIVE, I_MOULIN_AGGRESSIVE, D_MOULIN_AGGRESSIVE);
        motorEx.setRunMode(PositionControl);

    }

    // ========== POSITION CONTROL METHODS (same interface as original) ==========

    /**
     * Gets the next position in the sequence.
     */
    private static int getNextPosition(int pos) {
        return pos == MAX_POSITION ? MIN_POSITION : pos + 1;
    }

    /**
     * Gets the position n steps after the given position.
     * @param pos The starting position.
     * @param n The number of steps to advance.
     * @return The resulting position.
     */
    public static int getNNextPosition(int pos, int n) {
        int resultPos = pos;
        for (int i = 0; i < n; i++) {
            resultPos = getNextPosition(resultPos);
        }
        return resultPos;
    }

    /**
     * Gets the previous position in the sequence.
     */
    private static int getPreviousPosition(int pos) {
        return pos == MIN_POSITION ? MAX_POSITION : pos - 1;
    }

    /**
     * Gets the position n steps before the given position.
     * @param pos The starting position.
     * @param n The number of steps to go back.
     * @return The resulting position.
     */
    public static int getNPreviousPosition(int pos, int n) {
        int resultPos = pos;
        for (int i = 0; i < n; i++) {
            resultPos = getPreviousPosition(resultPos);
        }
        return resultPos;
    }

    /**
     * Forcibly sets the internal tracking of the moulin's position.
     * @param pos The new position to set.
     */
    public void hardSetPosition(int pos) {
        moulinPosition = anyIntToMoulinPosition(pos);
    }

    /**
     * Converts any integer to a valid moulin position (1-6).
     * @param intPos The integer to convert.
     * @return A valid moulin position.
     */
    public static int anyIntToMoulinPosition(int intPos) {
        int normalizedPos = ((intPos % TOTAL_POSITIONS) + TOTAL_POSITIONS) % TOTAL_POSITIONS;
        if (normalizedPos == 0) {
            return TOTAL_POSITIONS;
        }
        return normalizedPos;
    }

    /**
     * Sets the moulin to a specific position (1-6).
     * @param targetPos The target position.
     * @param makeShort Whether to take the shortest path.
     */
    public void setMoulinPosition(int targetPos, boolean makeShort) {
        if (!isValidPosition(targetPos)) {
            throw new IllegalArgumentException("Invalid moulin state: " + targetPos + ". Must be between "
                    + MIN_POSITION + " and " + MAX_POSITION);
        }

        int newTargetTicks;

        if (makeShort) {
            int optimalStateDifference = calculateOptimalStateDifference(moulinPosition, targetPos);
            newTargetTicks = (optimalStateDifference * INTERVALLE_TICKS_MOULIN);
        } else {
            int stateDifference = calculatePositivePositionDifference(moulinPosition, targetPos);
            newTargetTicks = (stateDifference * INTERVALLE_TICKS_MOULIN);
        }

        incrementTargetMotorPosition(newTargetTicks);
        moulinPosition = targetPos;
    }

    /**
     * Calculates the positive (clockwise) difference between two positions.
     * @param currentPos The current position.
     * @param targetPos The target position.
     * @return The number of positive steps to reach the target.
     */
    public static int calculatePositivePositionDifference(int currentPos, int targetPos) {
        if (targetPos >= currentPos) {
            return targetPos - currentPos;
        } else {
            return (MAX_POSITION - currentPos) + (targetPos - MIN_POSITION + 1);
        }
    }

    /**
     * Calculates the optimal (shortest) difference between two positions.
     * @param currentPos The current position.
     * @param targetPos The target position.
     * @return The shortest difference (positive for clockwise, negative for counter-clockwise).
     */
    public static int calculateOptimalStateDifference(int currentPos, int targetPos) {
        int directDifference = targetPos - currentPos;
        int positiveWrapDifference = directDifference - TOTAL_POSITIONS;
        int negativeWrapDifference = directDifference + TOTAL_POSITIONS;

        int optimalDifference = directDifference;

        if (Math.abs(negativeWrapDifference) < Math.abs(optimalDifference)) {
            optimalDifference = negativeWrapDifference;
        }

        if (Math.abs(positiveWrapDifference) < Math.abs(optimalDifference)) {
            optimalDifference = positiveWrapDifference;
        }

        return optimalDifference;
    }

    /**
     * Finds the closest position to the current shooting position from a list of possibilities.
     * @param possiblePositions An array of possible positions.
     * @return The closest position, or -1 if the array is empty.
     */
    public static int getClosestPositionToShoot(int[] possiblePositions) {
        if (possiblePositions == null || possiblePositions.length == 0) {
            return -1;
        }

        int shootingPosition = getOppositePosition(moulinPosition);
        int closestPos = possiblePositions[0];
        int smallestDistance = Math.abs(calculateOptimalStateDifference(shootingPosition, possiblePositions[0]));

        for (int i = 1; i < possiblePositions.length; i++) {
            int distance = Math.abs(calculateOptimalStateDifference(shootingPosition, possiblePositions[i]));
            if (distance < smallestDistance) {
                smallestDistance = distance;
                closestPos = possiblePositions[i];
            }
        }

        return closestPos;
    }

    /**
     * Gets the position opposite to the given position.
     * @param pos The position.
     * @return The opposite position.
     */
    public static int getOppositePosition(int pos) {
        return (pos + 3 - 1) % TOTAL_POSITIONS + 1;
    }

    private static boolean isValidPosition(int pos) {
        return pos >= MIN_POSITION && pos <= MAX_POSITION;
    }

    /**
     * Sets the target position for the motor.
     * @param position The target position in encoder ticks.
     */
    public void setTargetMotorPosition(int position) {
        motorEx.setTargetPosition(position);
    }

    /**
     * Increments the target motor position.
     * @param incr The increment in encoder ticks.
     */
    public void incrementTargetMotorPosition(double incr) {
        int newTargetPosition = (int) incr + getTargetMotorPosition();
        setTargetMotorPosition(newTargetPosition);
    }

    /**
     * Gets the target position of the motor.
     * @return The target position in encoder ticks.
     */
    public int getTargetMotorPosition() {
        return motorEx.motorEx.getTargetPosition();
    }

    /**
     * Checks if the motor is currently moving to a target.
     * @return True if the motor is busy, false otherwise.
     */
    public boolean isBusy() {
        return !motorEx.atTargetPosition();
    }

    /**
     * Sets the raw power of the motor.
     * @param power The power level (-1.0 to 1.0).
     */
    public void setPower(final double power) {
        motorEx.motorEx.setPower(power);
    }

    /**
     * Gets the current power of the motor.
     * @return The current power level.
     */
    public double getPower() {
        return motorEx.motorEx.getPower();
    }

    /**
     * Gets the current drawn by the motor.
     * @return The current in Amperes.
     */
    public double getVoltage(){
        return motorEx.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Gets the velocity of the motor.
     * @return The velocity in ticks per second.
     */
    public double getSpeed(){
        return motorEx.getVelocity();
    }

    /**
     * Gets the corrected velocity of the motor (if supported).
     * @return The corrected velocity.
     */
    public double getCorrectedSpeed(){
        return motorEx.getCorrectedVelocity();
    }

    /**
     * Gets the acceleration of the motor (if supported).
     * @return The acceleration.
     */
    public double getAcceleration(){
        return motorEx.getAcceleration();
    }

    /**
     * Sets the target position tolerance for the motor.
     * @param error The allowed error in encoder ticks.
     */
    public void setTargetMotorPositionTolerance(int error) {
        motorEx.setPositionTolerance(error); // allowed maximum error
    }

    /**
     * Gets the current logical position of the moulin (1-6).
     * @return The current position.
     */
    public int getPosition() {
        return moulinPosition;
    }

    /**
     * Gets the current position of the motor in encoder ticks.
     * @return The current motor position.
     */
    public int getMotorPosition() {
        return motorEx.getCurrentPosition();
    }

    /**
     * Gets the remaining distance to the target position.
     * @return The remaining distance in encoder ticks.
     */
    public int getRemainingDistance() {
        return getTargetMotorPosition() - getMotorPosition();
    }

    /**
     * Checks if the motor is over-current.
     * @return True if over-current, false otherwise.
     */
    public boolean isOverCurrent() {
        return motorEx.motorEx.isOverCurrent();
    }

    /**
     * Determines if the motor power should be cut.
     * @return True if the motor is over-current or has stabilized.
     */
    public boolean shouldStopPower() {
        return isOverCurrent() || (!isBusy() && Math.abs(getSpeed()) < 10 && getRemainingDistance() < 2 * MOULIN_POSITION_TOLERANCE);
    }

    /**
     * Gets the storage-specific index (1, 2, or 3) for a given moulin storage position.
     * @param pos A moulin position that must be a storage position (1, 3, or 5).
     * @return The corresponding storage index (1, 2, or 3).
     */
    public static int getMoulinStoragePos(int pos) {
        if (pos == 1) {
            return 1;
        } else if (pos == 3) {
            return 2;
        } else if (pos == 5) {
            return 3;
        } else {
            throw new IllegalArgumentException("Cannot return the storage pos, pos is not a storage :" + pos);
        }
    }

    /**
     * Gets the storage-specific index from a given moulin shooting position.
     * @param moulinPos A moulin position that must be a shooting position (2, 4, or 6).
     * @return The corresponding storage index (1, 2, or 3).
     */
    public static int getStoragePositionFromShootingPosition(int moulinPos) {
        if (moulinPos == 2) {
            return 3;
        } else if (moulinPos == 4) {
            return 1;
        } else if (moulinPos == 6) {
            return 2;
        } else {
            throw new IllegalArgumentException(
                    "Cannot return the storage pos, moulinPos is not a shooting pos for any pos :" + moulinPos);
        }
    }

    /**
     * Checks if a position is a storage position (1, 3, 5).
     * @param pos The position to check.
     * @return True if it's a storage position.
     */
    public static boolean isStoragePosition(int pos) {
        return pos == 1 || pos == 3 || pos == 5;
    }

    /**
     * Checks if a position is a shooting position (2, 4, 6).
     * @param pos The position to check.
     * @return True if it's a shooting position.
     */
    public static boolean isShootingPosition(int pos) {
        return pos == 2 || pos == 4 || pos == 6;
    }

    /**
     * Gets the shooting position for a given storage position.
     * @param storagePosition The storage position (1, 3, or 5).
     * @return The corresponding shooting position.
     */
    public static int getShootingPositionFor(int storagePosition) {
        if (isShootingPosition(storagePosition)) {
            throw new IllegalArgumentException("Invalid storage pos: " + storagePosition);
        }
        return getOppositePosition(storagePosition);
    }

    /**
     * Gets the storage position for a given shooting position.
     * @param shootingPosition The shooting position (2, 4, or 6).
     * @return The corresponding storage position.
     */
    public static int getStoragePositionFor(int shootingPosition) {
        if (isStoragePosition(shootingPosition)) {
            throw new IllegalArgumentException("Invalid shooting pos: " + shootingPosition);
        }
        return getOppositePosition(shootingPosition);
    }

    /**
     * Sets the PID coefficients for the motor's velocity control.
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     */
    public void setPID(double p, double i, double d) {
        motorEx.setVeloCoefficients(p, i, d);
        motorEx.setPositionCoefficient(p);
    }

    /**
     * Gets the current velocity PID coefficients.
     * @return The PID coefficients.
     */
    public PIDCoefficients getPID() {
        double[] pid = motorEx.getVeloCoefficients();
        return new PIDCoefficients(pid[0], pid[1], pid[2]);
    }

    /**
     * Sets the feedforward coefficients for the motor.
     * @param ks The static friction feedforward coefficient.
     * @param kv The velocity feedforward coefficient.
     * @param ka The acceleration feedforward coefficient.
     */
    public void setFF(double ks, double kv, double ka){
        motorEx.setFeedforwardCoefficients(ks, kv, ka);
    }

    /**
     * Gets the current feedforward coefficients.
     * @return An array containing the ks, kv, and ka coefficients.
     */
    public double[] getFF(){
        return motorEx.getFeedforwardCoefficients();
    }
}
