package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CS1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_GREEN_BLUE_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_GREEN_GREEN_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_GREEN_RED_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_PURPLE_BLUE_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_PURPLE_GREEN_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DETECT_PURPLE_RED_RGB;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_HUE_HIGHER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GREEN_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MARGIN_GREEN_RGB_DETECTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MARGIN_PURPLE_RGB_DETECTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GAIN_DETECTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PURPLE_HUE_HIGHER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PURPLE_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SAMPLE_SIZE_TEST;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name ="BasicColorSensor - Dinitech", group = "Basic")
public class BasicColorSensor extends GornetixRobotBase {
    private NormalizedColorSensor colorSensor1;
    // Store the last predominant colors for each sensor
    // Store the last predominant colors for each sensor
    public String predominantColor1 = null;
    public String predominantColor2 = null;
    public String predominantColor3 = null;
    public String predominantColor4 = null;

    // Store the last hue predominant colors for each sensor
    public String predominantHueColor1 = null;
    public String predominantHueColor2 = null;
    public String predominantHueColor3 = null;
    public String predominantHueColor4 = null;

    private NormalizedRGBA colors = null;

    private double hue;
    private double saturation;
    private double brightness;

    // Listes pour les moyennes glissantes (20 acquisitions)
    private List<Double> redSamples = new ArrayList<>();
    private List<Double> greenSamples = new ArrayList<>();
    private List<Double> blueSamples = new ArrayList<>();
    private List<Double> alphaSamples = new ArrayList<>();
    private List<Double> hueSamples = new ArrayList<>();
    private List<Double> saturationSamples = new ArrayList<>();
    private List<Double> brightnessSamples = new ArrayList<>();


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, CS1_NAME);
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();

        if ((float) GAIN_DETECTION != colorSensor1.getGain()){
            colorSensor1.setGain((float) GAIN_DETECTION);
        }

        updateSensor(colorSensor1, 1);

        // Ajouter les nouvelles valeurs aux listes
        addSampleToList(redSamples, colors.red);
        addSampleToList(greenSamples, colors.green);
        addSampleToList(blueSamples, colors.blue);
        addSampleToList(alphaSamples, colors.alpha);
        addSampleToList(hueSamples, hue);
        addSampleToList(saturationSamples, saturation);
        addSampleToList(brightnessSamples, brightness);

        // Calculer et afficher les moyennes
        telemetry.addData("Gain ", colorSensor1.getGain());
        telemetry.addLine("RGBA (Moyennes sur " + Math.min(redSamples.size(), SAMPLE_SIZE_TEST) + " acquisitions)");
        telemetry.addData("Red (avg)", String.format("%.4f", calculateAverage(redSamples)));
        telemetry.addData("Green (avg)", String.format("%.4f", calculateAverage(greenSamples)));
        telemetry.addData("Blue (avg)", String.format("%.4f", calculateAverage(blueSamples)));
        telemetry.addData("Alpha (avg)", String.format("%.4f", calculateAverage(alphaSamples)));

        telemetry.addLine("HSV (Moyennes sur " + Math.min(hueSamples.size(), SAMPLE_SIZE_TEST) + " acquisitions)");
        telemetry.addData("Hue (avg)", String.format("%.2f", calculateAverage(hueSamples)));
        telemetry.addData("Saturation (avg)", String.format("%.4f", calculateAverage(saturationSamples)));
        telemetry.addData("Brightness (avg)", String.format("%.4f", calculateAverage(brightnessSamples)));

        // Affichage des valeurs instantanées pour comparaison
//        telemetry.addLine("Valeurs instantanées");
//        telemetry.addData("Red (inst)", String.format("%.4f", colors.red));
//        telemetry.addData("Green (inst)", String.format("%.4f", colors.green));
//        telemetry.addData("Blue (inst)", String.format("%.4f", colors.blue));
//        telemetry.addData("Alpha (inst)", String.format("%.4f", colors.alpha));
//
//        telemetry.addData("Hue (inst)", String.format("%.2f", hue));
//        telemetry.addData("Saturation (inst)", String.format("%.4f", saturation));
//        telemetry.addData("Brightness (inst)", String.format("%.4f", brightness));

        telemetry.addData("green", isGreen());
        telemetry.addData("purple", isPurple());


//        if (isGreen(1)){
//            telemetry.addLine("CS1 Detects Green");
//        } else if (isPurple(1)){
//            telemetry.addLine("CS1 Detects Purple");
//        }
//
//        if (isGreenHue(1)){
//            telemetry.addLine("CS1 Detects Hue Green");
//        } else if (isPurpleHue(1)){
//            telemetry.addLine("CS1 Detects Hue Purple");
//        }
//
//        if(isDistanceLower(colorSensor1, 1)){
//            telemetry.addLine("CS1 7cm threshold");
//        }
//
//        telemetry.addData("colorSensor distance", getDistance(colorSensor1));
    }
    
    private boolean isHueBetween(double lowerThreshold, double upperThreshold) {
        double averageHue = calculateAverage(hueSamples);
        return averageHue >= lowerThreshold && averageHue <= upperThreshold;
    }

    public boolean isGreen(){
        return isHueBetween(GREEN_HUE_LOWER, GREEN_HUE_HIGHER) && calculateAverage(greenSamples) > calculateAverage(blueSamples);
    }

    public boolean isPurple(){
        return isHueBetween(PURPLE_HUE_LOWER, PURPLE_HUE_HIGHER) && calculateAverage(blueSamples) > calculateAverage(greenSamples);
    }

    
    public boolean isGreen(int sensorNumber){
        switch (sensorNumber) {
            case 1:
                return "Green".equals(predominantColor1);
            case 2:
                return "Green".equals(predominantColor2);
            case 3:
                return "Green".equals(predominantColor3);
            case 4:
                return "Green".equals(predominantColor4);
            default:
                return false;
        }
    }

    public boolean isGreenHue(int sensorNumber){
        switch (sensorNumber) {
            case 1:
                return "Green".equals(predominantHueColor1);
            case 2:
                return "Green".equals(predominantHueColor2);
            case 3:
                return "Green".equals(predominantHueColor3);
            case 4:
                return "Green".equals(predominantHueColor4);
            default:
                return false;
        }
    }

    public boolean isPurple(int sensorNumber){
        switch (sensorNumber) {
            case 1:
                return "Purple".equals(predominantColor1);
            case 2:
                return "Purple".equals(predominantColor2);
            case 3:
                return "Purple".equals(predominantColor3);
            case 4:
                return "Purple".equals(predominantColor4);
            default:
                return false;
        }
    }
    public boolean isPurpleHue(int sensorNumber){
        switch (sensorNumber) {
            case 1:
                return "Purple".equals(predominantHueColor1);
            case 2:
                return "Purple".equals(predominantHueColor2);
            case 3:
                return "Purple".equals(predominantHueColor3);
            case 4:
                return "Purple".equals(predominantHueColor4);
            default:
                return false;
        }
    }

    public void updateSensor(NormalizedColorSensor sensor, int sensorNumber){
        if (sensor == null) return;

        try {
            colors = sensor.getNormalizedColors();
            String predominantColor;

            float[] hsvValues = new float[3];

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

            hue = hsvValues[0];
            saturation = hsvValues[1];
            brightness = hsvValues[2];

            // HSV
            String predominantHueColor = detectHue(hue);

            // RGBA Determine the name of the color with the highest color value
            if (detectPurpleRGBA(colors)) {
                predominantColor = "Purple";
            } else if (detectGreenRGBA(colors)) {
                predominantColor = "Green";
            } else {
                predominantColor = "Other";
            }

            // Store the result in the appropriate field
            switch (sensorNumber) {
                case 1: predominantColor1 = predominantColor;
                    predominantHueColor1 = predominantHueColor;
                    break;
                case 2: predominantColor2 = predominantColor;
                    predominantHueColor2 = predominantHueColor;
                    break;
                case 3: predominantColor3 = predominantColor;
                    predominantHueColor3 = predominantHueColor;
                    break;
                case 4: predominantColor4 = predominantColor;
                    predominantHueColor4 = predominantHueColor;
                    break;
            }
        } catch (Exception ignored) {

        }
    }

    public boolean isDistanceLower(NormalizedColorSensor sensor, double distanceCM){
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM) < distanceCM;
    }

    private void updateGain(NormalizedColorSensor sensor, float gain){
        sensor.setGain(gain);
    }
    /** Detect the predominant color based on hue value
     * @return the predominant color via hue filter
     */
    private String detectHue(double hue) {
        if(hue < 30){
            return "Red";
        }
        else if (hue < 60) {
            return "Orange";
        }
        else if (hue < 90){
            return "Yellow";
        }
        else if (hue < 150){
            return "Green";
        }
        else if (hue < 225){
            return "Blue";
        }
        else if (hue < 350){
            return "Purple";
        }
        else{
            return "Red";
        }
    }

    /**
     * Detect purple from a rgba normalized (0 - 1) color input
     * @param colors
     * rgb purple is (177, 156, 217) -> normalized (0.694, 0.612, 0.851)
     * @return true if purple is detected, false otherwise
     */
    public boolean detectPurpleRGBA(NormalizedRGBA colors) {
        return (Math.abs(colors.red - DETECT_PURPLE_RED_RGB) < MARGIN_PURPLE_RGB_DETECTION) &&
                (Math.abs(colors.green - DETECT_PURPLE_GREEN_RGB) < MARGIN_PURPLE_RGB_DETECTION) &&
                (Math.abs(colors.blue - DETECT_PURPLE_BLUE_RGB) < MARGIN_PURPLE_RGB_DETECTION);
    }

    /**
     * Detect purple from a rgba normalized (0 - 1) color input
     * @param colors
     * rgb purple is (177, 156, 217) -> normalized (0.694, 0.612, 0.851)
     * @return true if purple is detected, false otherwise
     */
    public boolean detectGreenRGBA(NormalizedRGBA colors) {
        return (Math.abs(colors.red - DETECT_GREEN_RED_RGB) < MARGIN_GREEN_RGB_DETECTION) &&
                (Math.abs(colors.green - DETECT_GREEN_GREEN_RGB) < MARGIN_GREEN_RGB_DETECTION) &&
                (Math.abs(colors.blue - DETECT_GREEN_BLUE_RGB) < MARGIN_GREEN_RGB_DETECTION);
    }

    public double getDistance(NormalizedColorSensor sensor){
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
    }

    /**
     * Ajoute un échantillon à la liste et maintient la taille à SAMPLE_SIZE
     * @param list la liste à mettre à jour
     * @param value la nouvelle valeur à ajouter
     */
    private void addSampleToList(List<Double> list, double value) {
        list.add(value);
        
        // Si la liste dépasse 21 éléments, on garde seulement le plus récent
        if (list.size() > SAMPLE_SIZE_TEST) {
            // Supprime l'élément le plus ancien (index 0)
            list.remove(0);
        }
    }

    /**
     * Calcule la moyenne des valeurs dans la liste
     * @param list la liste des valeurs
     * @return la moyenne des valeurs
     */
    private double calculateAverage(List<Double> list) {
        if (list.isEmpty()) {
            return 0.0;
        }
        
        double sum = 0.0;
        for (double value : list) {
            sum += value;
        }
        
        return sum / list.size();
    }
}
