package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAGNETIC_ON_MOULIN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OFFSET_MAGNETIC_POS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_TELE_INCREMENT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.MagneticSwitch;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.MoulinVelocityControlled;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.TripleColorSensors;

import java.util.Arrays;

/**
 * An alternative implementation of the Trieur (sorter) subsystem that uses velocity control for the moulin.
 * <p>
 * This subsystem provides the same functionality as {@link TrieurSubsystem} but utilizes a
 * {@link MoulinVelocityControlled} instance for more advanced motor control, including tunable PID and
 * feedforward gains. This allows for potentially smoother and more accurate positioning of the sorting mechanism.
 *
 * @see TrieurSubsystem
 * @see MoulinVelocityControlled
 */
public class VelocityTrieurSubsystem extends SubsystemBase {
    /**
     * Represents the possible colors of an artifact.
     */
    public enum ArtifactColor {
        NONE, GREEN, PURPLE, UNKNOWN
    }

    private final Trappe trappe;
    private final MoulinVelocityControlled moulin;
    private final TripleColorSensors tripleColorSensors;
    private final MagneticSwitch magneticSwitch;
    private final Telemetry telemetry;

    private final ArtifactColor[] moulinStoragePositionColors = new ArtifactColor[3];
    private ArtifactColor lastDetectedColor = ArtifactColor.NONE;

    private boolean newRegister = false;
    private boolean newColoredRegister = false;
    private boolean isFull = false;
    private boolean wentRecalibrationOpposite = true;

    /**
     * Constructs a new VelocityTrieurSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public VelocityTrieurSubsystem(HardwareMap hardwareMap, final Telemetry telemetry) {
        trappe = new Trappe(hardwareMap);
        moulin = new MoulinVelocityControlled(hardwareMap);
        tripleColorSensors = new TripleColorSensors(hardwareMap);
        magneticSwitch = new MagneticSwitch(hardwareMap);

        this.telemetry = telemetry;

        clearAllStoredColors();
        setWentRecalibrationOpposite(true);
    }

    // Methods for controlling and querying the state of the trieur components

    public void rotateToMoulinPosition(int pos, boolean makeShort) {
        moulin.setMoulinPosition(pos, makeShort);
    }

    public int getMoulinPosition() {
        return moulin.getPosition();
    }

    public int getMoulinMotorPosition() {
        return moulin.getMotorPosition();
    }

    public int getMoulinMotorTargetPosition() {
        return moulin.getTargetMotorPosition();
    }

    public int getMoulinMotorRemainingDistance() {
        return moulin.getRemainingDistance();
    }

    public void resetTargetMoulinMotor() {
        setMoulinTargetPosition(getMoulinMotorPosition());
    }

    public void setMoulinPower(double power) {
        moulin.setPower(power);
    }

    private double getMoulinSpeed(){
        return moulin.getSpeed();
    }

    public void incrementMoulinTargetPosition(double incr) {
        moulin.incrementTargetMotorPosition(incr);
    }

    public void setMoulinTargetPosition(int targetPosition) {
        moulin.setTargetMotorPosition(targetPosition);
    }

    public boolean shouldMoulinStopPower() {
        return moulin.shouldStopPower();
    }

    public int getNNextMoulinPosition(int pos, int n) {
        return Moulin.getNNextPosition(pos, n);
    }

    public int getNPreviousMoulinPosition(int pos, int n) {
        return Moulin.getNPreviousPosition(pos, n);
    }

    public boolean isArtefactInTrieur() {
        return tripleColorSensors.isDistanceBetween(1, DISTANCE_ARTEFACT_IN_TRIEUR, DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR)
                || tripleColorSensors.isDistanceBetween(2, DISTANCE_ARTEFACT_IN_TRIEUR,
                        DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR);
    }

    public void updateColorSensors() {
        tripleColorSensors.updateAllSensors();
    }

    public void clearSamplesColorSensors() {
        tripleColorSensors.clearSamplesAllSensors();
    }

    public int getColorSensorSampleCount() {
        return tripleColorSensors.getSampleCount(1);
    }

    public void registerArtefact() {
        ArtifactColor detectedColor = detectBottomArtifactColor();
        int bottomStoragePos = Moulin.getMoulinStoragePos(getMoulinPosition());

        setMoulinStoragePositionColor(bottomStoragePos, detectedColor);

        if (detectedColor == ArtifactColor.GREEN || detectedColor == ArtifactColor.PURPLE) {
            setNewcoloredRegister(true);
        }
        setNewRegister(true);

        if (howManyArtifacts() == 3) {
            setIsFull(true);
        }

        lastDetectedColor = detectedColor;
    }

    private ArtifactColor detectBottomArtifactColor() {
        boolean cs1Purple = tripleColorSensors.isPurple(1);
        boolean cs2Purple = tripleColorSensors.isPurple(2);

        if ((cs1Purple && cs2Purple) || (cs1Purple && !cs2Purple) || (cs2Purple && !cs1Purple)) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.GREEN;
    }

    public void setMoulinStoragePositionColor(int pos, ArtifactColor color) {
        moulinStoragePositionColors[pos - 1] = color;
    }

    public ArtifactColor getMoulinStoragePositionColor(int pos) {
        return moulinStoragePositionColors[pos - 1];
    }

    public void clearMoulinStoragePositionColor(int pos) {
        if (!Moulin.isStoragePosition(pos)) {
            throw new IllegalArgumentException("Invalid storage pos: " + pos);
        }
        setMoulinStoragePositionColor(pos, ArtifactColor.NONE);
        setIsFull(false);
    }

    public void clearAllStoredColors() {
        Arrays.fill(moulinStoragePositionColors, ArtifactColor.NONE);
        setIsFull(false);
    }

    public ArtifactColor getLastDetectedColor() {
        return lastDetectedColor;
    }

    public int[] getPosWithColor(ArtifactColor color) {
        int[] tempPos = new int[3];
        int count = 0;

        for (int i = 0; i < moulinStoragePositionColors.length; i++) {
            if (moulinStoragePositionColors[i] == color) {
                tempPos[count++] = (i * 2) + 1;
            }
        }

        return Arrays.copyOf(tempPos, count);
    }

    public boolean wentRecalibrationOpposite() {
        return wentRecalibrationOpposite;
    }

    public void setWentRecalibrationOpposite(boolean wentRecalibrationOpposite) {
        this.wentRecalibrationOpposite = wentRecalibrationOpposite;
    }

    public int getClosestShootingPositionForColor(ArtifactColor color) {
        int[] posWithColor = getPosWithColor(color);

        for (int i = 0; i < posWithColor.length; i++) {
            posWithColor[i] = Moulin.getOppositePosition(posWithColor[i]);
        }

        if (posWithColor.length > 0) {
            return Moulin.getClosestPositionToShoot(posWithColor);
        }
        return -1;
    }

    public int getClosestShootingPositionAnyColor() {
        int[] greenPositions = getPosWithColor(ArtifactColor.GREEN);
        int[] purplePositions = getPosWithColor(ArtifactColor.PURPLE);
        int[] unknownPositions = getPosWithColor(ArtifactColor.UNKNOWN);

        int totalSize = greenPositions.length + purplePositions.length + unknownPositions.length;
        int[] anyColorPositions = new int[totalSize];
        int index = 0;

        for (int pos : greenPositions) anyColorPositions[index++] = Moulin.getOppositePosition(pos);
        for (int pos : purplePositions) anyColorPositions[index++] = Moulin.getOppositePosition(pos);
        for (int pos : unknownPositions) anyColorPositions[index++] = Moulin.getOppositePosition(pos);

        return Moulin.getClosestPositionToShoot(anyColorPositions);
    }

    public boolean isMoulinOverCurrent() {
        return moulin.isOverCurrent();
    }

    public void closeTrappe() {
        trappe.close();
    }

    public void openTrappe() {
        trappe.open();
    }

    public void incrOpenTrappe() {
        trappe.incrementalRotation(TRAPPE_TELE_INCREMENT);
    }

    public void incrCloseTrappe() {
        trappe.incrementalRotation(-TRAPPE_TELE_INCREMENT);
    }

    public boolean isTrappeOpen(){
        return trappe.isDoorOpen();
    }

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

    public void setIsFull(boolean newIsFull) {
        isFull = newIsFull;
    }

    public boolean getIsFull() {
        return isFull;
    }

    public int howManyArtifacts() {
        int count = 0;
        for (ArtifactColor color : moulinStoragePositionColors) {
            if (color != ArtifactColor.NONE) count++;
        }
        return count;
    }

    private boolean isMoulinBusy() {
        return moulin.isBusy();
    }

    private void powerMoulinIfNeeded() {
        if (isMoulinBusy()) {
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
        }
    }

    private void recalibrateMoulin() {
        int remainingDistance = Math.abs(getMoulinMotorRemainingDistance());
        double intervals = (double) remainingDistance / INTERVALLE_TICKS_MOULIN;
        int intPartOfRounded = (int) Math.round(intervals);
        double differenceToIntRounded = intervals - intPartOfRounded;
        int diffTicks = (int) Math.round(Math.abs(differenceToIntRounded) * INTERVALLE_TICKS_MOULIN);

        if (diffTicks != 0) {
            int adjustment = (differenceToIntRounded > 0) ? -diffTicks : diffTicks;
            incrementMoulinTargetPosition(adjustment + OFFSET_MAGNETIC_POS);
        }

        moulin.hardSetPosition(MAGNETIC_ON_MOULIN_POSITION + intPartOfRounded);
    }

    public void hardSetMoulinPosition(int pos){
        moulin.hardSetPosition(pos);
    }

    public void setPID(double p, double i, double d) {
        moulin.setPID(p, i, d);
    }

    public PIDCoefficients getPID() {
        return moulin.getPID();
    }

    public void setFF(double kS, double kV, double kA) {
        moulin.setFF(kS, kV, kA);
    }

    public double[] getFF() {
        return moulin.getFF();
    }

    @Override
    public void periodic() {
        powerMoulinIfNeeded();
        printMoulinTelemetry(telemetry);
    }

    private void printMoulinTelemetry(final Telemetry telemetry) {
        telemetry.addData("Moulin Ticks", getMoulinMotorPosition());
        telemetry.addData("Moulin Target", getMoulinMotorTargetPosition());
        telemetry.addData("Moulin Speed", getMoulinSpeed());
        telemetry.addData("Moulin Position", getMoulinPosition());
        telemetry.addData("Moulin At Target", !isMoulinBusy());
    }
}
