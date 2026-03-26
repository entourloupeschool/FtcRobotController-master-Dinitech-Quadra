package org.firstinspires.ftc.teamcode.dinitech.other;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.cmToInch;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;
import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
public class RobotDefinitions {
    public static double ROBOT_LENGTH_CM = 42.95;
    public static double ROBOT_LENGTH_INCH = cmToInch(ROBOT_LENGTH_CM); // = 16.91
    public static double ROBOT_WIDTH_CM = 44.8;
    public static double ROBOT_WIDTH_INCH = cmToInch(ROBOT_WIDTH_CM); // = 17.638
    public static double ROBOT_LENGTH_CHARGEUR_CM = 44.95;
    public static double ROBOT_LENGTH_CHARGEUR_INCH = cmToInch(ROBOT_LENGTH_CHARGEUR_CM);
}
