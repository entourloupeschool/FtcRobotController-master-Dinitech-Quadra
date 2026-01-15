package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_ON_MOULIN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OFFSET_MAGNETIC_POS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_TELE_INCREMENT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.TripleColorSensors;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.MagneticSwitch;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

import java.util.Arrays;

/**
 * A command-based subsystem for managing the Trieur (sorter) mechanism.
 * <p>
 * This subsystem orchestrates the complex interactions between the {@link Moulin} (rotating carousel),
 * {@link Trappe} (door), {@link TripleColorSensors}, and a {@link MagneticSwitch} to create a complete
 * automated sorting system for game artifacts.
 * <p>
 * Key responsibilities include:
 * <ul>
 *     <li>Controlling the rotation and positioning of the Moulin.</li>
 *     <li>Detecting the presence and color of artifacts.</li>
 *     <li>Tracking the contents of each Moulin storage slot.</li>
 *     <li>Recalibrating the Moulin's position using the magnetic switch.</li>
 *     <li>Providing high-level methods for finding and shooting specific colored artifacts.</li>
 * </ul>
 */
public class TrieurSubsystem extends SubsystemBase {
    /**
     * Represents the possible colors of an artifact.
     */
    public enum ArtifactColor {
        NONE,   // No artifact
        GREEN,  // Green artifact
        PURPLE, // Purple artifact
        UNKNOWN // Artifact of an undetermined color
    }

    private final Trappe trappe;
    private final Moulin moulin;
    private final TripleColorSensors tripleColorSensors;
    private final MagneticSwitch magneticSwitch;
    private final Telemetry telemetry;

    private final ArtifactColor[] moulinStoragePositionColors = new ArtifactColor[3];
    private ArtifactColor lastDetectedColor = ArtifactColor.NONE;

    private boolean newRegister = false;
    private boolean newColoredRegister = false;
    private boolean isFull = false;
    private boolean wentRecalibrationOpposite = true;
    
    private boolean overCurrentOccurredDuringRotation = false;

    /**
     * Constructs a new TrieurSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public TrieurSubsystem(HardwareMap hardwareMap, final Telemetry telemetry) {
        trappe = new Trappe(hardwareMap);
        moulin = new Moulin(hardwareMap);
        tripleColorSensors = new TripleColorSensors(hardwareMap);
        magneticSwitch = new MagneticSwitch(hardwareMap);

        this.telemetry = telemetry;

        clearAllStoredColors();
        setWentRecalibrationOpposite(true);

//        setMoulinPower(POWER_MOULIN_ROTATION);
    }

    /**
     * Rotates the moulin to a specific position.
     * @param pos The target position (1-6).
     * @param makeShort Whether to take the shortest path.
     */
    public void rotateToMoulinPosition(int pos, boolean makeShort) {
        moulin.setMoulinPosition(pos, makeShort);
    }

    /**
     * Gets the current logical position of the moulin (1-6).
     * @return The current moulin position.
     */
    public int getMoulinPosition() {
        return moulin.getPosition();
    }

    /**
     * Gets the current encoder position of the moulin motor.
     * @return The motor position in ticks.
     */
    public int getMoulinMotorPosition() {
        return moulin.getMotorPosition();
    }

    /**
     * Gets the target encoder position of the moulin motor.
     * @return The target motor position in ticks.
     */
    public int getMoulinMotorTargetPosition() {
        return moulin.getTargetMotorPosition();
    }

    /**
     * Gets the remaining distance to the target position for the moulin motor.
     * @return The remaining distance in ticks.
     */
    public int getMoulinMotorRemainingDistance() {
        return moulin.getRemainingDistance();
    }

    /**
     * Resets the moulin motor's target to its current position, stopping any movement.
     */
    public void resetTargetMoulinMotor() {
        setMoulinTargetPosition(getMoulinMotorPosition());
    }

    /**
     * Sets the raw power of the moulin motor.
     * @param power The power level (-1.0 to 1.0).
     */
    public void setMoulinPower(double power) {
        moulin.setPower(power);
    }
    
    private double getMoulinSpeed(){
        return moulin.getSpeed();
    }

    /**
     * Increments the moulin motor's target position.
     * @param incr The increment in ticks.
     */
    public void incrementMoulinTargetPosition(double incr) {
        moulin.incrementTargetMotorPosition(incr);
    }

    /**
     * Sets the moulin motor's target position directly.
     * @param targetPosition The target position in ticks.
     */
    public void setMoulinTargetPosition(int targetPosition) {
        moulin.setTargetPositionMotor(targetPosition);
    }

    /**
     * Checks if the moulin has stabilized at its target position.
     * @return True if stabilized, false otherwise.
     */
    public boolean isMoulinTargetStabilized(){
        return moulin.isTargetStabilized();
    }
    
    /**
     * Determines if the moulin motor's power should be cut.
     * @return True if power should be stopped.
     */
    public boolean shouldMoulinStopPower() {
        return moulin.shouldStopPower();
    }

    /**
     * Gets the moulin position n steps after the given position.
     * @param pos The starting position.
     * @param n The number of steps to advance.
     * @return The resulting position.
     */
    public int getNNextMoulinPosition(int pos, int n) {
        return Moulin.getNNextPosition(pos, n);
    }

    /**
     * Gets the moulin position n steps before the given position.
     * @param pos The starting position.
     * @param n The number of steps to go back.
     * @return The resulting position.
     */
    public int getNPreviousMoulinPosition(int pos, int n) {
        return Moulin.getNPreviousPosition(pos, n);
    }

    /**
     * Checks if an artifact is detected within the intake area of the sorter.
     *
     * @return True if an artifact is detected, false otherwise.
     */
    public boolean isArtefactInTrieur() {
        return tripleColorSensors.isDistanceBetween(1, DISTANCE_ARTEFACT_IN_TRIEUR, DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR)
                || tripleColorSensors.isDistanceBetween(2, DISTANCE_ARTEFACT_IN_TRIEUR,
                        DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR);
    }

    /**
     * Updates the readings for all color sensors.
     */
    public void updateColorSensors() {
        tripleColorSensors.updateAllSensors();
    }

    /**
     * Clears the collected samples for all color sensors.
     */
    public void clearSamplesColorSensors() {
        tripleColorSensors.clearSamplesAllSensors();
    }

    /**
     * Gets the number of samples collected by the color sensors for averaging.
     * @return The sample count.
     */
    public int getColorSensorSampleCount() {
        return tripleColorSensors.getSampleCount(1);
    }

    /**
     * Detects the color of an artifact at the intake and registers it in the corresponding
     * moulin storage slot.
     */
    public void registerArtefact() {
        ArtifactColor detectedColor = detectBottomArtifactColor();

        // Store the color at the current moulin storage Pos
        setMoulinStoragePositionColor(Moulin.getMoulinStoragePos(getMoulinPosition()), detectedColor);

        if (detectedColor == ArtifactColor.GREEN || detectedColor == ArtifactColor.PURPLE) {
            setNewcoloredRegister(true);
        }
        setNewRegister(true);

        lastDetectedColor = detectedColor;
    }

    /**
     * Detects the color of the artifact at the pickup position.
     *
     * @return The detected artifact color.
     */
    private ArtifactColor detectBottomArtifactColor() {
        boolean cs1Purple = tripleColorSensors.isPurple(1);
        boolean cs1Neither = tripleColorSensors.isNeither(1);

        boolean cs2Purple = tripleColorSensors.isPurple(2);
        boolean cs2Neither = tripleColorSensors.isNeither(2);

        if ((cs1Purple && cs2Purple) || (cs1Purple && cs2Neither) || (cs2Purple && cs1Neither)) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.GREEN;
    }

    /**
     * More strictly detects the color of the artifact at the pickup position.
     *
     * @return The detected artifact color, or UNKNOWN if uncertain.
     */
    private ArtifactColor hardDetectBottomArtifactColor() {
        boolean cs1Green = tripleColorSensors.isGreen(1);
        boolean cs1Purple = tripleColorSensors.isPurple(1);

        boolean cs2Green = tripleColorSensors.isGreen(2);
        boolean cs2Purple = tripleColorSensors.isPurple(2);

        if ((cs1Green && cs2Green) || (cs1Green && !cs2Purple) || (cs2Green && !cs1Purple)) {
            return ArtifactColor.GREEN;
        } else if ((cs1Purple && cs2Purple) || (cs1Purple && !cs2Green) || (cs2Purple && !cs1Green)) {
            return ArtifactColor.PURPLE;
        }
        return ArtifactColor.UNKNOWN;
    }


    /**
     * Sets the color associated with a specific moulin storage position.
     *
     * @param pos   The moulin storage position (1-3).
     * @param color The artifact color to associate.
     */
    public void setMoulinStoragePositionColor(int pos, ArtifactColor color) {
        moulinStoragePositionColors[pos - 1] = color;

        if (howManyArtifacts() == 3) {
            setIsFull(true);
        }
    }

    /**
     * Gets the color associated with a specific moulin storage position.
     *
     * @param pos The moulin storage position (1-3).
     * @return The artifact color at that position.
     */
    public ArtifactColor getMoulinStoragePositionColor(int pos) {
        return moulinStoragePositionColors[pos - 1];
    }

    /**
     * Clears the color association for a storage slot, typically after an artifact is shot.
     *
     * @param pos The moulin storage position (1-3) to clear.
     */
    public void clearMoulinStoragePositionColor(int pos) {
        if (!Moulin.isStoragePosition(pos)) {
            throw new IllegalArgumentException("Invalid storage pos: " + pos);
        }
        setMoulinStoragePositionColor(pos, ArtifactColor.NONE);

        setIsFull(false);
    }

    /**
     * Clears all stored artifact colors, resetting the moulin to an empty state.
     */
    public void clearAllStoredColors() {
        Arrays.fill(moulinStoragePositionColors, ArtifactColor.NONE);
        setIsFull(false);
    }

    /**
     * Gets the last detected artifact color.
     *
     * @return The color of the last detected artifact.
     */
    public ArtifactColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    /**
     * Gets all moulin storage positions that contain artifacts of a specific color.
     *
     * @param color The color to search for.
     * @return An array of moulin storage positions containing that color.
     */
    public int[] getPosWithColor(ArtifactColor color) {
        int[] tempPos = new int[3];
        int count = 0;

        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
            if (moulinStoragePositionColors[i] == color) {
                tempPos[count] = (i * 2) + 1;
                count++;
            }
        }

        int[] result = new int[count];
        System.arraycopy(tempPos, 0, result, 0, count);
        return result;
    }

    public boolean wentRecalibrationOpposite() {
        return wentRecalibrationOpposite;
    }

    public void setWentRecalibrationOpposite(boolean wentRecalibrationOpposite) {
        this.wentRecalibrationOpposite = wentRecalibrationOpposite;
    }

    /**
     * Finds the closest shooting position for an artifact of a specific color.
     *
     * @param color The desired color.
     * @return The closest shooting position (2, 4, or 6), or -1 if no artifacts of that color are available.
     */
    public int getClosestShootingPositionForColor(ArtifactColor color) {
        int[] posWithColor = getPosWithColor(color);

        // Transform storage positions to shooting positions
        for (int i = 0; i < posWithColor.length; i++) {
            posWithColor[i] = Moulin.getOppositePosition(posWithColor[i]);
        }

        if (posWithColor.length > 0) {
            return Moulin.getClosestPositionToShoot(getMoulinPosition(), posWithColor);
        }
        return -1;
    }

    /**
     * Finds the closest shooting position for any available artifact.
     *
     * @return The closest shooting position for any color, or -1 if the moulin is empty.
     */
    public int getClosestShootingPositionAnyColor() {
        int[] anyColorPositions = new int[3];
        int greenShootingPosition = getClosestShootingPositionForColor(ArtifactColor.GREEN);
        int purpleShootingPosition = getClosestShootingPositionForColor(ArtifactColor.PURPLE);
        int unknownShootingPosition = getClosestShootingPositionForColor(ArtifactColor.UNKNOWN);

        int index = 0;
        if (greenShootingPosition != -1) anyColorPositions[index++] = greenShootingPosition;
        if (purpleShootingPosition != -1) anyColorPositions[index++] = purpleShootingPosition;
        if (unknownShootingPosition != -1) anyColorPositions[index++] = unknownShootingPosition;
        
        int[] availablePositions = Arrays.copyOf(anyColorPositions, index);

        return Moulin.getClosestPositionToShoot(getMoulinPosition(), availablePositions);
    }

    /**
     * Checks if the moulin motor is currently experiencing an over-current condition.
     * @return True if over-current is detected.
     */
    public boolean isMoulinOverCurrent() {
        return moulin.isOverCurrent();
    }

    /**
     * Opens the trappe.
     */
    public void closeTrappe() {
        trappe.close();
    }

    /**
     * Closes the trappe.
     */
    public void openTrappe() {
        trappe.open();
    }

    /**
     * Incrementally opens the trappe.
     */
    public void incrOpenTrappe() {
        incrTrappe(TRAPPE_TELE_INCREMENT);
    }

    /**
     * Incrementally closes the trappe.
     */
    public void incrCloseTrappe() {
        incrTrappe(-TRAPPE_TELE_INCREMENT);
    }

    /**
     * Incrementally moves the trappe.
     * @param increment The amount to move the trappe servo.
     */
    public void incrTrappe(double increment){
        trappe.incrementalRotation(increment);
    }

    /**
     * Checks if the trappe is open.
     * @return True if the trappe is open.
     */
    public boolean isTrappeOpen(){
        return trappe.isDoorOpen();
    }

    /**
     * Checks if the magnetic switch is activated.
     * @return True if the switch is pressed.
     */
    public boolean isMagneticSwitch() {
        return magneticSwitch.isMagnetic();
    }

    public void setNewRegister(boolean register) {
        newRegister = register;
    }

    public boolean hasNewRegister() {
        return newRegister;
    }

    public void setNewcoloredRegister(boolean coloredRegister) {
        newColoredRegister = coloredRegister;
    }

    public boolean hasNewColoredRegister() {
        return newColoredRegister;
    }

    /**
     * Sets the full state of the trieur.
     * @param newIsFull The new full state.
     */
    public void setIsFull(boolean newIsFull) {
        isFull = newIsFull;
    }

    /**
     * Gets the full state of the trieur.
     * @return True if the trieur is full.
     */
    public boolean getIsFull() {
        return isFull;
    }

    /**
     * Counts the number of artifacts currently stored in the moulin.
     * @return The number of stored artifacts (0-3).
     */
    public int howManyArtifacts() {
        int count = 0;
        for (ArtifactColor color : moulinStoragePositionColors) {
            if (color != ArtifactColor.NONE) {
                count++;
            }
        }
        return count;
    }

    private boolean isMoulinBusy() {
        return moulin.isBusy();
    }

    private double getMoulinVoltage(){
        return moulin.getVoltage();
    }

    /**
     * Manages power to the moulin motor and triggers recalibration when necessary.
     * This should be called periodically.
     */
    private void moulinLogic() {
        if (isMoulinTargetStabilized()) {
            setMoulinPower(0);
            setNewRegister(false);
            setNewcoloredRegister(false);

            if (wentRecalibrationOpposite() && isMagneticSwitch()) {
                recalibrateMoulin();
                setWentRecalibrationOpposite(false);
            }

        } else {
            if (getMoulinPosition() != MAGNETIC_ON_MOULIN_POSITION) {
                setWentRecalibrationOpposite(true);
            }

            if (isMoulinOverCurrent()) {
                setMoulinPower(0);
                telemetry.addLine("Moulin Over Current");
                return;
            }

            setMoulinPower(POWER_MOULIN_ROTATION);
        }
    }

    /**
     * Recalibrates the moulin's target position and logical position based on the magnetic switch activation.
     */
    private void recalibrateMoulin() {
        int remainingDistance = Math.abs(getMoulinMotorRemainingDistance());
        double intervals = (double) remainingDistance / INTERVALLE_TICKS_MOULIN;
        int intPartOfRounded = (int) Math.round(intervals);
        double differenceToIntRounded = (intervals - intPartOfRounded);
        double absDiff = Math.abs(differenceToIntRounded);
        int diffTicks = (int) Math.round(absDiff * INTERVALLE_TICKS_MOULIN);

        if (diffTicks != 0) {
            if (differenceToIntRounded > 0) {
                incrementMoulinTargetPosition(-diffTicks + OFFSET_MAGNETIC_POS);
            } else {
                incrementMoulinTargetPosition(diffTicks + OFFSET_MAGNETIC_POS);
            }
        }

        moulin.hardSetPosition(MAGNETIC_ON_MOULIN_POSITION + intPartOfRounded);
    }

    /**
     * Forcibly sets the moulin's internal logical position.
     * @param pos The new position.
     */
    public void hardSetMoulinPosition(int pos){
        moulin.hardSetPosition(pos);
    }

    /**
     * Sets the PIDF coefficients for the moulin motor.
     * @param p The proportional coefficient.
     * @param i The integral coefficient.
     * @param d The derivative coefficient.
     * @param f The feedforward coefficient.
     */
    public void setPIDF(double p, double i, double d, double f) {
        moulin.setPIDF(p, i, d, f);
    }

    /**
     * Gets the PIDF coefficients for the moulin motor.
     * @return The PIDF coefficients.
     */
    public PIDFCoefficients getPIDF() {
        return moulin.getPIDF();
    }

    @Override
    public void periodic() {
        moulinLogic();
//        printMagneticTelemetry(telemetry);
//        printMoulinTelemetry(telemetry);
        printStoredArtifactsTelemetry(telemetry);
//        printDistanceTelemetry(telemetry);
//        printColorTelemetry(telemetry);
    }

    private void printTrappeTelemetry(final Telemetry telemetry) {
        telemetry.addData("trappe ouverte ?", trappe.isDoorOpen());
    }

    private void printMagneticTelemetry(final Telemetry telemetry) {
        telemetry.addData("magnticSwitch", magneticSwitch.isMagnetic());
    }

    private void printMoulinTelemetry(final Telemetry telemetry) {
        telemetry.addData("moulin ticks", getMoulinMotorPosition());
        telemetry.addData("moulinTarget", getMoulinMotorTargetPosition());
//        telemetry.addData("moulinSpeed", getMoulinSpeed());
//        telemetry.addData("moulin position", getMoulinPosition());
        telemetry.addData("moulin at target", !isMoulinBusy());
    }

    private void printColorTelemetry(final Telemetry telemetry) {
        tripleColorSensors.updateAllSensors();

        for (int i = 1; i <= 3; i++) {
            telemetry.addData("Blue " + i, String.format("%.4f", tripleColorSensors.getAverageBlue(i)));
            telemetry.addData("Red " + i, String.format("%.4f", tripleColorSensors.getAverageRed(i)));
            telemetry.addData("Green " + i, String.format("%.4f",tripleColorSensors.getAverageGreen(i)));
            telemetry.addData("Hue " + i, String.format("%.4f", tripleColorSensors.getAverageHue(i)));
            telemetry.addData("Sat " + i, String.format("%.4f", tripleColorSensors.getAverageSaturation(i)));
        }
        if (tripleColorSensors.isGreen(1)) telemetry.addLine("CS1 Detects Green");
        else if (tripleColorSensors.isPurple(1)) telemetry.addLine("CS1 Detects Purple");

        if (tripleColorSensors.isGreen(2)) telemetry.addLine("CS2 Detects Green");
        else if (tripleColorSensors.isPurple(2)) telemetry.addLine("CS2 Detects Purple");
    }

    private void printDistanceTelemetry(final Telemetry telemetry) {
        telemetry.addData("Artefact in Trieur", isArtefactInTrieur());
        telemetry.addData("CS1 Distance", tripleColorSensors.getDistance(1));
        telemetry.addData("CS2 Distance", tripleColorSensors.getDistance(2));
    }

    private void printStoredArtifactsTelemetry(final Telemetry telemetry) {
        telemetry.addLine("=== Stored Artifacts ===");
        for (int i = 1; i <= 3; i++) {
            ArtifactColor color = getMoulinStoragePositionColor(i);
            if (color != ArtifactColor.NONE) {
                telemetry.addData("Storage position " + i, color.toString());
            }
        }

        telemetry.addData("Last registered color", getLastDetectedColor());
        telemetry.addData("isFull", getIsFull());
        telemetry.addData("Green shooting positions", getClosestShootingPositionForColor(ArtifactColor.GREEN));
        telemetry.addData("Purple shooting positions", getClosestShootingPositionForColor(ArtifactColor.PURPLE));
    }
}
