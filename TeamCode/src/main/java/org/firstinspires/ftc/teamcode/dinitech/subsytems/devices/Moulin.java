package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.D_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.F_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.I_MOULIN_AGGRESSIVE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.P_MOULIN_AGGRESSIVE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Represents the Moulin mechanism, a rotating system with 6 discrete positions for handling game elements.
 *
 * <p>The Moulin is modeled as a circular system with 6 slots, each 60 degrees apart.
 * There are two main types of positions:
 * <ul>
 *     <li><b>Storage Positions (1, 3, 5):</b> Used for loading game elements into the mechanism.</li>
 *     <li><b>Shooting Positions (2, 4, 6):</b> Used for launching game elements from the mechanism.</li>
 * </ul>
 *
 * <p>The class provides a comprehensive API for controlling the Moulin's rotation, including:
 * <ul>
 *     <li>Moving to specific positions.</li>
 *     <li>Calculating the shortest path for rotation.</li>
 *     <li>Converting between storage and shooting positions.</li>
 *     <li>Querying the current state and position of the motor.</li>
 * </ul>
 *
 * <p><b>Position Mapping:</b>
 * <ul>
 *     <li>Position 1: Storage slot 1 is at the loading area.</li>
 *     <li>Position 2: Storage slot 3 is at the shooting area.</li>
 *     <li>Position 3: Storage slot 2 is at the loading area.</li>
 *     <li>Position 4: Storage slot 1 is at the shooting area.</li>
 *     <li>Position 5: Storage slot 3 is at the loading area.</li>
 *     <li>Position 6: Storage slot 2 is at the shooting area.</li>
 * </ul>
 */
public class Moulin {
    public static final int MIN_POSITION = 1;
    public static final int MAX_POSITION = 6;
    public static final int TOTAL_POSITIONS = 6;

    private final DcMotorEx dcMotorEx;
    private int moulinPosition;

    public Moulin(final HardwareMap hardwareMap) {
        dcMotorEx = hardwareMap.get(DcMotorEx.class, MOULIN_MOTOR_NAME);

        dcMotorEx.setZeroPowerBehavior(BRAKE);

        resetMoulin();

        setTargetPositionMotorTolerance(MOULIN_POSITION_TOLERANCE);
    }

    /**
     * Resets the Moulin to its initial state. This includes resetting the encoder,
     * setting the motor to RUN_TO_POSITION mode, and initializing PIDF coefficients.
     * The moulin position is set to 1.
     */
    public void resetMoulin() {
        dcMotorEx.setMode(STOP_AND_RESET_ENCODER);
        setTargetPositionMotor(getMotorPosition());
        dcMotorEx.setMode(RUN_TO_POSITION);
        setPIDF(P_MOULIN_AGGRESSIVE, I_MOULIN_AGGRESSIVE, D_MOULIN_AGGRESSIVE, F_MOULIN_AGGRESSIVE);
        moulinPosition = 1;
    }

    /**
     * Helper method to get the next state in the sequence
     */
    private static int getNextPosition(int pos) {
        if (isValidPosition(pos)){
            return pos == MAX_POSITION ? MIN_POSITION : pos + 1;
        } else {
            throw new IllegalArgumentException("Invalid position to compute next : " + pos);
        }
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
     * Helper method to get the previous state in the sequence
     */
    private static int getPreviousPosition(int pos) {
        if (isValidPosition(pos)){
            return pos == MIN_POSITION ? MAX_POSITION : pos - 1;
        } else {
            throw new IllegalArgumentException("Invalid position to compute previous : " + pos);
        }
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
     * This does not move the motor, but updates the software's understanding of the position.
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
        int normalizedPos = ((intPos % TOTAL_POSITIONS) + TOTAL_POSITIONS) % TOTAL_POSITIONS; // ensure positive modulo
        if (normalizedPos == 0) {
            return TOTAL_POSITIONS;
        }
        return normalizedPos;
    }

    /**
     * Set the moulin to a specific state (1-6)
     *
     * @param targetPos the target state (1-6)
     * @param makeShort if true, use shortest rotation path; if false, go through
     *                  all states in positive direction
     * @throws IllegalArgumentException if state is not between 1 and 6
     */
    public void setPosition(int targetPos, boolean makeShort) {
        if (!isValidPosition(targetPos)) {
            throw new IllegalArgumentException("Invalid moulin state: " + targetPos + ". Must be between "
                    + MIN_POSITION + " and " + MAX_POSITION);
        }

        int newTargetTicks;

        if (makeShort) {
            // Use shortest path logic - calculate relative to current position
            int optimalStateDifference = calculateOptimalStateDifference(moulinPosition, targetPos);

            // Calculate target position by moving the optimal number of steps from current
            // position
            newTargetTicks = (optimalStateDifference * INTERVALLE_TICKS_MOULIN);
        } else {
            // Go through all states in positive direction (clockwise)
            int stateDifference = calculatePositivePositionDifference(moulinPosition, targetPos);

            // Calculate target position by going the specified number of steps forward
            newTargetTicks = (stateDifference * INTERVALLE_TICKS_MOULIN);
        }

        incrementTargetMotorPosition(newTargetTicks);
        hardSetPosition(targetPos);
    }

    /**
     * Bypass the setPosition
     * full rotation of the moulin
     */
    public void revolution(){
        incrementTargetMotorPosition(INTERVALLE_TICKS_MOULIN * TOTAL_POSITIONS);
    }

    /**
     * Calculate the positive (clockwise) path between current and target state
     * Always goes in the positive direction through all states
     *
     * @param currentPos the current Position
     * @param targetPos  the target Position
     * @return the number of positive steps to reach target (always positive)
     */
    public static int calculatePositivePositionDifference(int currentPos, int targetPos) {
        if (targetPos >= currentPos) {
            // Direct path in positive direction
            return targetPos - currentPos;
        } else {
            // Wrap around: go to max pos, then continue to target
            // Example: from position 2 to position 1 = (6-2+1) + (1-1) = 5 steps
            return (MAX_POSITION - currentPos) + (targetPos - MIN_POSITION + 1);
        }
    }

    /**
     * Calculate the optimal path (shortest rotation) between current and target
     * state
     *
     * @param currentPos the current pos
     * @param targetPos  the target pos
     * @return the state difference using the shortest path (negative =
     *         counterclockwise, positive = clockwise)
     */
    public static int calculateOptimalStateDifference(int currentPos, int targetPos) {
        int directDifference = targetPos - currentPos;

        // Calculate alternative paths around the circle
        int positiveWrapDifference = directDifference - TOTAL_POSITIONS;
        int negativeWrapDifference = directDifference + TOTAL_POSITIONS;

        // Find the path with smallest absolute value (shortest rotation)
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
     * Find the closest position to the opposite current moulin position from an
     * array of possible positions
     *
     * @param possiblePositions Array of positions to choose from
     * @return The closest state to the opposite current position, or -1 if array is
     *         empty
     */
    public static int getClosestPositionToShoot(int currentPosition, int[] possiblePositions) {
        if (possiblePositions.length == 0) {
            throw new IllegalArgumentException("Trying to find closest position to shoot but possiblePositions is empty");
        }

        int shootingPosition = getOppositePosition(currentPosition);

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
     * Get the opposite position of a position (180 degrees rotation)
     *
     * @param pos a position
     * @return the opposite position
     */
    public static int getOppositePosition(int pos) {
        return (pos + 3 - 1) % TOTAL_POSITIONS + 1; // +3 for opposite, -1 and +1 for 1-based indexing
    }

    private static boolean isValidPosition(int pos) {
        return pos >= MIN_POSITION && pos <= MAX_POSITION;
    }

    /**
     * Sets the target position for the motor.
     * @param position The target position in encoder ticks.
     */
    public void setTargetPositionMotor(int position) {
        dcMotorEx.setTargetPosition(position);
    }

    /**
     * Increments the target motor position by a specified amount.
     * @param incr The amount to increment the position by, in encoder ticks.
     */
    public void incrementTargetMotorPosition(double incr) {
        int newTargetPosition = (int) incr + getTargetMotorPosition();
        setTargetPositionMotor(newTargetPosition);
    }

    /**
     * Gets the current target position of the motor.
     * @return The target position in encoder ticks.
     */
    public int getTargetMotorPosition() {
        return dcMotorEx.getTargetPosition();
    }
    
    /**
     * Checks if the motor is currently busy moving to a target position.
     * @return True if the motor is busy, false otherwise.
     */
    public boolean isBusy() {
        return dcMotorEx.isBusy();
    }

    /**
     * Sets the power of the motor.
     * @param power The power level, from -1.0 to 1.0.
     */
    public void setPower(final double power) {
        dcMotorEx.setPower(power);
    }

    /**
     * Gets the current power of the motor.
     * @return The current power level.
     */
    public double getPower() {
        return dcMotorEx.getPower();
    }

    /**
     * Gets the current drawn by the motor.
     * @return The current in Amperes.
     */
    public double getVoltage() {
        return dcMotorEx.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Gets the velocity of the motor.
     * @return The velocity in ticks per second.
     */
    public double getSpeed() {
        return dcMotorEx.getVelocity();
    }

    /**
     * Sets the target position tolerance for the motor.
     * @param error The allowed error in encoder ticks.
     */
    public void setTargetPositionMotorTolerance(int error) {
        dcMotorEx.setTargetPositionTolerance(error); // allowed maximum error
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
        return dcMotorEx.getCurrentPosition();
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
        return dcMotorEx.isOverCurrent();
    }

    /**
     * Checks if the motor has stabilized at its target position.
     * @return True if the motor is not busy, speed is low, and is within tolerance.
     */
    public boolean isTargetStabilized() {
//        return !isBusy() && Math.abs(getSpeed()) < 10 && Math.abs(getRemainingDistance()) <= MOULIN_POSITION_TOLERANCE;
        return !isBusy() && Math.abs(getSpeed()) < 10;
    }

    /**
     * Determines if the motor power should be cut.
     * @return True if the motor is over-current or has stabilized.
     */
    public boolean shouldStopPower() {
        return isOverCurrent() || isTargetStabilized();
    }


    /**
     * Check if a state is a storage position (1, 3, 5)
     *
     * @param pos the position to check
     * @return true if it's a storage position
     */
    public static boolean isStoragePosition(int pos) {
        return pos == 1 || pos == 3 || pos == 5;
    }

    /**
     * Check if a state is a shooting position (2, 4, 6)
     *
     * @param pos the position to check
     * @return true if it's a shooting position
     */
    public static boolean isShootingPosition(int pos) {
        return pos == 2 || pos == 4 || pos == 6;
    }


    /**
     * Sets the PIDF coefficients for the motor.
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     * @param f The feedforward coefficient.
     */
    public void setPIDF(double p, double i, double d, double f) {
        dcMotorEx.setVelocityPIDFCoefficients(p, i, d, f);
        dcMotorEx.setPositionPIDFCoefficients(p);
        // https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
    }

    /**
     * Gets the PIDF coefficients for the motor.
     * @return The PIDF coefficients.
     */
    public PIDFCoefficients getPIDF() {
        return dcMotorEx.getPIDFCoefficients(RUN_USING_ENCODER);
    }

}
