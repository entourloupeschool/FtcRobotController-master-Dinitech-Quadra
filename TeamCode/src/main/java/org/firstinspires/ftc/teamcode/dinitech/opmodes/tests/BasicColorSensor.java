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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.Gornetix;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name ="BasicColorSensor - Dinitech", group = "Basic")
public class BasicColorSensor extends Gornetix {



    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {



        super.run();

    }


    private void updateGain(NormalizedColorSensor sensor, float gain){
        sensor.setGain(gain);
    }
    /** Detect the predominant color based on hue value
     * @return the predominant color via hue filter
     */

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
