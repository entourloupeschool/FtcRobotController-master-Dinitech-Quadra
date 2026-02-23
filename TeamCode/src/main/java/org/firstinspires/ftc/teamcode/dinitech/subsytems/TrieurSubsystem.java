package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_ON_MOULIN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OFFSET_MAGNETIC_POS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OVER_CURRENT_BACKOFF_TICKS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION_OVERCURRENT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_TELE_INCREMENT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCorrectOverCurrent;
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
    private final TelemetryManager telemetryM;

    private final ArtifactColor[] moulinStoragePositionColors = new ArtifactColor[6];
    private ArtifactColor lastDetectedColor = ArtifactColor.NONE;

    private int overcurrentCounts = 0;

    public void setOvercurrentCounts(int overcurrentCounts) {
        overcurrentCounts = overcurrentCounts;
    }

    public int getOvercurrentCounts() {
        return overcurrentCounts;
    }

    private boolean newRegister = false;
    private boolean newColoredRegister = false;
    private boolean isFull = false;
    private boolean wentRecalibrationOpposite = true;
    private boolean wantsMotifShoot = false;

    /**
     * Constructs a new TrieurSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetryM   The telemetryM object for logging.
     */
    public TrieurSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        trappe = new Trappe(hardwareMap);
        moulin = new Moulin(hardwareMap);
        tripleColorSensors = new TripleColorSensors(hardwareMap);
        magneticSwitch = new MagneticSwitch(hardwareMap);

        this.telemetryM = telemetryM;

        clearAllStoredColors();
        setWentRecalibrationOpposite(true);
        setWantsMotifShoot(false);
    }

    /**
     * Rotates the moulin to a specific position.
     * @param pos The target position (1-6).
     * @param makeShort Whether to take the shortest path.
     */
    public void moulinToPosition(int pos, boolean makeShort) {
        moulin.setPosition(pos, makeShort);
    }

    /**
     * Full rotation of the moulin
     */
    public void moulinRevolution(){
        moulin.revolution();
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
        setMoulinMotorTargetPosition(getMoulinMotorPosition());
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
    public void setMoulinMotorTargetPosition(int targetPosition) {
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
     * Determines if the moulin motor's power should be cut. With Loose condition.
     * @return True if power should be stopped.
     */
    public boolean shouldMoulinStopPowerLoose() {
        return moulin.shouldStopPowerLoose();
    }

    /**
     * Determines if the moulin motor's power should be cut. With Loose condition.
     * @return True if power should be stopped.
     */
    public boolean shouldMoulinStopPowerVeryLoose() {
        return moulin.shouldStopPowerVeryLoose();
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
        return tripleColorSensors.isDistanceBetween(1, DISTANCE_ARTEFACT_IN_TRIEUR * SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF, DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR)
                || tripleColorSensors.isDistanceBetween(2, DISTANCE_ARTEFACT_IN_TRIEUR * SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF,
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
        setMoulinStoragePositionColor(getMoulinPosition(), detectedColor);

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
     * @param pos   The moulin storage position (1-6).
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
    public int getPosWithColor(ArtifactColor color) {

        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
            if (moulinStoragePositionColors[i] == color) return i + 1;
        }
        return -1;
    }

    public int getPosColorClosestShoot(ArtifactColor color) {
        // Collect all positions with the specified color (max 3 storage positions)
        int[] positionsWithColor = new int[3];
        int count = 0;

        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
            if (moulinStoragePositionColors[i] == color) {
                positionsWithColor[count++] = Moulin.getOppositePosition(i + 1);
            }
        }

        if (count == 0) {
            return -1;
        }

        int[] availablePositions = Arrays.copyOf(positionsWithColor, count);
        return Moulin.getClosestPositionToShoot(getMoulinPosition(), availablePositions);
    }


    public boolean wentRecalibrationOpposite() {
        return wentRecalibrationOpposite;
    }

    public void setWentRecalibrationOpposite(boolean wentRecalibrationOpposite) {
        this.wentRecalibrationOpposite = wentRecalibrationOpposite;
    }

    public boolean wantsMotifShoot() {
        return wantsMotifShoot;
    }

    public void setWantsMotifShoot(boolean wantsMotifShoot) {
        this.wantsMotifShoot = wantsMotifShoot;
    }

    /**
     * Finds the closest shooting position for an artifact of a specific color.
     *
     * @param color The desired color.
     * @return The closest shooting position (2, 4, or 6), or -1 if no artifacts of that color are available.
     */
    public int getClosestShootingPositionForColor(ArtifactColor color) {
        int posWithColor = getPosWithColor(color);

        if (posWithColor == -1) {
            return posWithColor;
        }

        return Moulin.getOppositePosition(posWithColor);
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

        } else {
            if (isMoulinOverCurrent()) {

                if (getOvercurrentCounts() == 0 && getCurrentCommand() != null){
                    getCurrentCommand().cancel();
                    new MoulinCorrectOverCurrent(this).schedule();
                }
                setOvercurrentCounts(getOvercurrentCounts() + 1);
                if (getOvercurrentCounts() > 10){
                    setMoulinPower(0);
                }

                telemetryM.addLine("Moulin Over Current");
                return;
            }

            setOvercurrentCounts(0);

            setMoulinPower(POWER_MOULIN_ROTATION);
        }

        if (!isMagneticSwitch()){
            setWentRecalibrationOpposite(true);
        } else if (isMagneticSwitch() && wentRecalibrationOpposite()) {
            recalibrateMoulin();
            setWentRecalibrationOpposite(false);
        }
    }

    /**
     * Recalibrates the moulin's target position and logical position based on the magnetic switch activation.
     */
    private void recalibrateMoulin() {
        // Get the absolute value of the remaining distance to the target position
        int remainingDistance = Math.abs(getMoulinMotorRemainingDistance());
        
        // Calculate how many intervals of INTERVALLE_TICKS_MOULIN fit into the remaining distance
        double intervals = (double) remainingDistance / INTERVALLE_TICKS_MOULIN;
        
        // Round the number of intervals to the nearest integer
        int intPartOfRounded = (int) Math.round(intervals);
        
        // Calculate the fractional difference between the actual intervals and the rounded value
        double differenceToIntRounded = (intervals - intPartOfRounded);
        
        // Get the absolute value of the fractional difference
        double absDiff = Math.abs(differenceToIntRounded);
        
        // Convert the fractional difference back to ticks for fine adjustment
        int diffTicks = (int) Math.round(absDiff * INTERVALLE_TICKS_MOULIN);

        // If there's a fine adjustment needed, apply it in the appropriate direction
        if (diffTicks != 0) {
            if (differenceToIntRounded > 0) {
                // If we rounded down, subtract the difference to correct the target position
                incrementMoulinTargetPosition(OFFSET_MAGNETIC_POS - diffTicks);
            } else {
                // If we rounded up, add the difference to correct the target position
                incrementMoulinTargetPosition(OFFSET_MAGNETIC_POS + diffTicks);
            }
        }
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
        printMagneticTelemetryManager(telemetryM);
        printMoulinTelemetryManager(telemetryM);
//        printStoredArtifactsTelemetryManager(telemetryM);
//        printDistanceTelemetryManager(telemetryM);
//        printColorTelemetryManager(telemetryM);
    }

    private void printTrappeTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("trappe ouverte ?", trappe.isDoorOpen());
    }

    private void printMagneticTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("magnticSwitch", magneticSwitch.isMagnetic());
    }

    private void printMoulinTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("moulin ticks", getMoulinMotorPosition());
        telemetryM.addData("moulinTarget", getMoulinMotorTargetPosition());
//        telemetryM.addData("moulinSpeed", getMoulinSpeed());
        telemetryM.addData("moulin position", getMoulinPosition());
//        telemetryM.addData("moulin at target", !isMoulinBusy());
//        telemetryM.addData("shootGreenPos", getClosestShootingPositionForColor(ArtifactColor.GREEN));
//        telemetryM.addData("shootPurplePos", getClosestShootingPositionForColor(ArtifactColor.PURPLE));
        telemetryM.addLine("getPosWithColor");
        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
            telemetryM.addData(String.valueOf(i), moulinStoragePositionColors[i]);
        }
    }

    private void printColorTelemetryManager(final TelemetryManager telemetryM) {
        tripleColorSensors.updateAllSensors();

        for (int i = 1; i <= 3; i++) {
            telemetryM.addData("Blue " + i, String.format("%.4f", tripleColorSensors.getAverageBlue(i)));
            telemetryM.addData("Red " + i, String.format("%.4f", tripleColorSensors.getAverageRed(i)));
            telemetryM.addData("Green " + i, String.format("%.4f",tripleColorSensors.getAverageGreen(i)));
            telemetryM.addData("Hue " + i, String.format("%.4f", tripleColorSensors.getAverageHue(i)));
            telemetryM.addData("Sat " + i, String.format("%.4f", tripleColorSensors.getAverageSaturation(i)));
        }
        if (tripleColorSensors.isGreen(1)) telemetryM.addLine("CS1 Detects Green");
        else if (tripleColorSensors.isPurple(1)) telemetryM.addLine("CS1 Detects Purple");

        if (tripleColorSensors.isGreen(2)) telemetryM.addLine("CS2 Detects Green");
        else if (tripleColorSensors.isPurple(2)) telemetryM.addLine("CS2 Detects Purple");
    }

    private void printDistanceTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("Artefact in Trieur", isArtefactInTrieur());
        telemetryM.addData("CS1 Distance", tripleColorSensors.getDistance(1));
        telemetryM.addData("CS2 Distance", tripleColorSensors.getDistance(2));
    }

    private void printStoredArtifactsTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addLine("=== Stored Artifacts ===");
        for (int i = 1; i <= 3; i++) {
            ArtifactColor color = getMoulinStoragePositionColor(i);
            if (color != ArtifactColor.NONE) {
                telemetryM.addData("Storage position " + i, color.toString());
            }
        }

        telemetryM.addData("Last registered color", getLastDetectedColor());
        telemetryM.addData("isFull", getIsFull());
        telemetryM.addData("Green shooting positions", getClosestShootingPositionForColor(ArtifactColor.GREEN));
        telemetryM.addData("Purple shooting positions", getClosestShootingPositionForColor(ArtifactColor.PURPLE));
    }
}
