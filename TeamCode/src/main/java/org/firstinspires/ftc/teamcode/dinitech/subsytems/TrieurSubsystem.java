package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.DISTANCE_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.INTERVALLE_TICKS_MOULIN_DOUBLE;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.MAX_OVERCURRENT_COUNT;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.OFFSET_MAGNETIC_POS;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.POWER_MOULIN_ROTATION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.POWER_SCALER_RECALIBRATION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.SCALE_RECALIBRATION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.getDegreesFromTicks;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
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
@Configurable
public class TrieurSubsystem extends SubsystemBase {
    public static final int MODE_RAMASSAGE_TELE_TIMEOUT = 300;
    public static final int MODE_RAMASSAGE_AUTO_TIMEOUT = 23;

    public void resetMoulinEncoderTarget() {
        moulin.resetEncoderTarget();
    }

    public void setMoulinEncoderTargetPosition(double targetCorrection) {
        moulin.setTargetEncoderPos(targetCorrection);
    }

    /**
     * Represents the possible colors of an artifact.
     */
    public enum ArtifactColor {
        IMP,    // NOT A STORAGE POS
        NONE,   // No artifact
        GREEN,  // Green artifact
        PURPLE, // Purple artifact
        UNKNOWN // Artifact of an undetermined color
    }

    private final Trappe trappe;
    private final Moulin moulin;
    private final Globals.RunningAverage lastMoulinMotorTicks = new Globals.RunningAverage(5);

    private final TripleColorSensors tripleColorSensors;
    private final MagneticSwitch magneticSwitch;
    private final TelemetryManager telemetryM;

    private final ArtifactColor[] moulinStoragePositionColors = new ArtifactColor[6];

    public ArtifactColor[] getMoulinStoragePositionColors(){
        return moulinStoragePositionColors;
    }

    private ArtifactColor lastDetectedColor = ArtifactColor.NONE;
    private int overcurrentCounts = 0;

    private int howManyArtefacts = 0;
    private double lastRecalibrationIncrement = 0;

    public int getHowManyArtefacts() {
        return howManyArtefacts;
    }

    public void setHowManyArtefacts(int howManyArtefacts) {
        this.howManyArtefacts = howManyArtefacts;
    }

    private int detectionTimeout;

    public void setDetectionTimeout(int detectionTimeout) {
        this.detectionTimeout = detectionTimeout;
    }

    public int getDetectionTimeout() {
        return detectionTimeout;
    }

    public void setOvercurrentCounts(int overcurrentCounts) {
        overcurrentCounts = overcurrentCounts;
    }

    public int getOvercurrentCounts() {
        return overcurrentCounts;
    }

    private boolean newRegister = false;
    private boolean newColoredRegister = false;
    private boolean wentRecalibrationOpposite = true;
    private boolean hasInitCalibration;
    
    public boolean hasInitCalibration() {
        return hasInitCalibration;
    }
    public void setHasInitCalibration(boolean hasInitCalibration) {
        this.hasInitCalibration = hasInitCalibration;
    }

    private boolean wantsMotifShoot;

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
        setDetectionTimeout(MODE_RAMASSAGE_TELE_TIMEOUT);
        setHowManyArtefacts(0);
        setHasInitCalibration(false);

//        setMoulinPower(POWER_MOULIN_ROTATION);
    }

    /**
     * Rotates the moulin to a specific position.
     * @param pos The target position (1-6).
     * @param makeShort Whether to take the shortest path.
     */
    public void moulinRotateToPosition(int pos, boolean makeShort) {
        moulin.rotateToPosition(pos, makeShort);
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

    public double getMoulinEncoderTargetPosition(){
        return moulin.getTargetEncoderPos();
    }

    /**
     * Gets the remaining distance to the target position for the moulin motor.
     * @return The remaining distance in ticks.
     */
    public int getMoulinMotorRemainingTicks() {
        return moulin.getRemainingTicks();
    }

    public double getMoulinMotorTargetTicks() {
        return moulin.getTargetTick();
    }
    public double getMoulinMotorRemainingTicksDouble(){
        return getMoulinMotorTargetTicks() - getMoulinMotorPosition();
    }
    public double getMoulinMotorRemainingDegrees() {
        return getDegreesFromTicks(getMoulinMotorRemainingTicks());
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

    /**
     * Increments the moulin motor's target position.
     * @param incr The increment in ticks.
     */
    public void incrementMoulinTargetPosition(double incr) {
        moulin.incrementTargetMotorPosition(incr);
    }

    public void incrementMoulinEncoderTargetPosition(double incr) {
        moulin.incrementTargetEncoderPos(incr);
    }

    /**
     * Sets the moulin motor's target position directly.
     * @param targetPosition The target position in ticks.
     */
    public void setMoulinMotorTargetPosition(double targetPosition) {
        moulin.setTargetMotorPosition(targetPosition);
    }
    
    /**
     * Determines if the moulin motor's power should be cut.
     * @return True if power should be stopped.
     */
    public boolean shouldMoulinStopPower() {
        return moulin.shouldStopPower();
    }

    public boolean isMoulinEncoderCloseToTarget(double margin){
        return moulin.isEncoderCloseToTarget(margin);
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
        setHowManyArtefacts(getHowManyArtefacts() + 1);

        ArtifactColor detectedColor = detectBottomArtifactColor();
        clearSamplesColorSensors();


        // Store the color at the current moulin storage Pos
        setMoulinStoragePositionColor(getMoulinPosition(), detectedColor);

        if (detectedColor == ArtifactColor.GREEN || detectedColor == ArtifactColor.PURPLE) {
            setNewColoredRegister(true);
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
     * Sets the color associated with a specific moulin storage position.
     *
     * @param pos   The moulin storage position (1-6).
     * @param color The artifact color to associate.
     */
    public void setMoulinStoragePositionColor(int pos, ArtifactColor color) {
        moulinStoragePositionColors[pos - 1] = color;
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
        setHowManyArtefacts(getHowManyArtefacts() - 1);
    }

    /**
     * Clears all stored artifact colors, resetting the moulin to an empty state.
     */
    public void clearAllStoredColors() {
        moulinStoragePositionColors[0] = ArtifactColor.NONE;
        moulinStoragePositionColors[2] = ArtifactColor.NONE;
        moulinStoragePositionColors[4] = ArtifactColor.NONE;
        moulinStoragePositionColors[1] = ArtifactColor.IMP;
        moulinStoragePositionColors[3] = ArtifactColor.IMP;
        moulinStoragePositionColors[5] = ArtifactColor.IMP;

        setHowManyArtefacts(0);
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
    public void toggleTrappe() {
        trappe.toggleTrappe();
    }

    /**
     * Incrementally opens the trappe.
     */
    public void incrOpenTrappe() {
        incrTrappe(Trappe.TRAPPE_TELE_INCREMENT);
    }

    /**
     * Incrementally closes the trappe.
     */
    public void incrCloseTrappe() {
        incrTrappe(-Trappe.TRAPPE_TELE_INCREMENT);
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
        return trappe.getTrappeIsOpen();
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
    public boolean getNewRegister(){
        return newRegister;
    }


    public void setNewColoredRegister(boolean coloredRegister) {
        newColoredRegister = coloredRegister;
    }


    public boolean isFull() {
        return getHowManyArtefacts() == 3;
    }
    
    public boolean isEmpty(){return getHowManyArtefacts() == 0;}

    private boolean isMoulinBusy() {
        return moulin.isBusy();
    }

    private boolean isMoulinRestAtTarget(){
        return !isMoulinBusy() && lastMoulinMotorTicks.getStd() == 0;
    }


    /**
     * Manages power to the moulin motor and triggers recalibration when necessary.
     * This should be called periodically.
     */
    private void moulinLogic() {
        lastMoulinMotorTicks.add(getMoulinMotorPosition());

        moulin.setPower(moulin.getPIDFRun(getMoulinMotorPosition()));

        if (isMoulinOverCurrent()) {

                setOvercurrentCounts(getOvercurrentCounts() + 1);

//                if (getOvercurrentCounts() > MAX_OVERCURRENT_COUNT){
//                    setMoulinPower(0);
//                    resetMoulinEncoderTarget();
//                }

                telemetryM.addLine("Moulin Over Current");
                return;

        } else {
            setOvercurrentCounts(0);

        }

        if (!isMagneticSwitch()){
            setWentRecalibrationOpposite(true);

        } else if (isMagneticSwitch() && wentRecalibrationOpposite() && hasInitCalibration()) {
            setWentRecalibrationOpposite(false);
            recalibrateMoulin();

        }
    }

    /**
     * Recalibrates the moulin's target position and logical position based on the magnetic switch activation.
     */
    private void recalibrateMoulin() {
        // Get the absolute value of the remaining distance to the target position
        // remove the offset of the magnetic switch
        double remainingTicks = Math.max(0, Math.abs(getMoulinMotorRemainingTicksDouble()) - OFFSET_MAGNETIC_POS);
        if (remainingTicks == 0) return;

        // Calculate how many intervals of INTERVALLE_TICKS_MOULIN fit into the remaining distance
        double intervals = remainingTicks / INTERVALLE_TICKS_MOULIN_DOUBLE;
        
        // Round the number of intervals to the nearest integer
        int intPartOfRounded = (int) Math.round(intervals);
        
        // Calculate the fractional difference between the actual intervals and the rounded value
        double differenceToIntRounded = (intervals - intPartOfRounded);

        // Convert the fractional difference back to ticks for fine adjustment
        double diffTicks = Math.abs(differenceToIntRounded) * INTERVALLE_TICKS_MOULIN_DOUBLE;
        lastRecalibrationIncrement = diffTicks;

        double scaler = Math.pow(Math.min(diffTicks/SCALE_RECALIBRATION, 0.9), POWER_SCALER_RECALIBRATION);

        incrementMoulinEncoderTargetPosition(differenceToIntRounded > 0 ? - diffTicks * scaler : diffTicks * scaler);
    }

    /**
     * Forcibly sets the moulin's internal logical position.
     * @param pos The new position.
     */
    public void setMoulinPosition(int pos){
        moulin.setPosition(pos);
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
        telemetryM.addData("wantsMotif", wantsMotifShoot());

//        printMagneticTelemetryManager(telemetryM);
        printMoulinTelemetryManager(telemetryM);
        printTrappeTelemetryManager(telemetryM);
        telemetryM.addData("overcurrentCounts", getOvercurrentCounts());
//        printStoredArtifactsTelemetryManager(telemetryM);
//        updateColorSensors();
//        printDistanceTelemetryManager(telemetryM);
//        printColorTelemetryManager(telemetryM);
    }

    private void printTrappeTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("trappe ouverte ?", trappe.getTrappeIsOpen());
    }

    private void printMagneticTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("magnticSwitch", magneticSwitch.isMagnetic());
    }

    private void printMoulinTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addData("moulin ticks", getMoulinMotorPosition());
        telemetryM.addData("moulinMotorTarget Double", moulin.getTargetTick());
//        telemetryM.addData("moulinMotorTarget Int", moulin.getTargetMotorPosition());
//        telemetryM.addData("moulinTarget", getMoulinMotorTargetPosition());
//        telemetryM.addData("moulinSpeed", getMoulinSpeed());
//        int currentPos = getMoulinPosition();
//        telemetryM.addData("current moulin pos", currentPos);
//        telemetryM.addData("is storage", Moulin.isStoragePosition(currentPos));
//        telemetryM.addData("is shoot", Moulin.isShootingPosition(currentPos));
//        telemetryM.addData("MoulinNNext 1", Moulin.getNNextPosition(currentPos, 1));
//        telemetryM.addData("MoulinNNext 2", Moulin.getNNextPosition(currentPos, 2));

//        telemetryM.addData("moulin at target", !isMoulinBusy());
//        telemetryM.addData("shootGreenPos", getClosestShootingPositionForColor(ArtifactColor.GREEN));
//        telemetryM.addData("shootPurplePos", getClosestShootingPositionForColor(ArtifactColor.PURPLE));
//        telemetryM.addLine("getPosWithColor");
//        telemetryM.addData("howMany", getHowManyArtefacts());
//        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
//            telemetryM.addData(String.valueOf(i), moulinStoragePositionColors[i]);
//        }
        telemetryM.addData("recalibration", lastRecalibrationIncrement);
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
        updateColorSensors();
        telemetryM.addData("Artefact in Trieur", isArtefactInTrieur());
        telemetryM.addData("CS1 Distance", tripleColorSensors.getDistance(1));
        telemetryM.addData("CS2 Distance", tripleColorSensors.getDistance(2));
    }

    private void printStoredArtifactsTelemetryManager(final TelemetryManager telemetryM) {
        telemetryM.addLine("=== Stored Artifacts ===");
        for (int i = 1; i <= 6; i++) {
            ArtifactColor color = getMoulinStoragePositionColor(i);
            if (color != ArtifactColor.NONE && color != ArtifactColor.IMP) {
                telemetryM.addData("Storage position " + i, color.toString());
            }
        }

        telemetryM.addData("Last registered color", getLastDetectedColor());
        telemetryM.addData("isFull", isFull());
        telemetryM.addData("Green shooting positions", getClosestShootingPositionForColor(ArtifactColor.GREEN));
        telemetryM.addData("Purple shooting positions", getClosestShootingPositionForColor(ArtifactColor.PURPLE));
    }
}
