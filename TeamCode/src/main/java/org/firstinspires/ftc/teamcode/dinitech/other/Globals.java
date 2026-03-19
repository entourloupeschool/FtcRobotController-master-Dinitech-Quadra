package org.firstinspires.ftc.teamcode.dinitech.other;


import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Dictionary;

@Configurable
public class Globals {
    public static final double FIELD_SIDE_LENGTH = 144.0;

    public static double ROBOT_LENGTH_CM = 42.95;
    public static double ROBOT_LENGTH_INCH = cmToInch(ROBOT_LENGTH_CM); // = 16.91
    public static double ROBOT_WIDTH_CM = 44.8;
    public static double ROBOT_WIDTH_INCH = cmToInch(ROBOT_WIDTH_CM); // = 17.638
    public static double ROBOT_LENGTH_CHARGEUR_CM = 44.95;
    public static double ROBOT_LENGTH_CHARGEUR_INCH = cmToInch(ROBOT_LENGTH_CHARGEUR_CM);


    public static long TELEMETRY_UPDATE_INTERVAL_MS = 500;
    public static double BLUE_TEAM_HEADING = Math.PI;

    public static double a_Pedro = 5.85; // 5.95;
    public static double b_Pedro = 930; // 1060;

    /**
     * Gives back a linear speed from a range in inches
     * @param rangeInch The range value in inches, positive.
     * @return The speed value, also positive
     */
    public static double linearSpeedFromPedroRange(double rangeInch) {
        return a_Pedro * rangeInch + b_Pedro;
    }

    /**
     * Auto Phase globals
     */
    public static final double AUTO_ROBOT_CONSTRAINTS = 1;
    public static final double BRAKING_STRENGTH_PEDRO_DINITECH = 0.7; // 0.75
    public static final double BRAKING_START_PEDRO_DINITECH = 1; // 1.4
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME = 0.81;
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT = 0.68;
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT = 0.55;


    public static final double TILE_DIM = 24;
    public static final double FOLLOWER_T_POSITION_END = 0.91;
    public static final double LENGTH_X_ROW = TILE_DIM * 0.78;
    public static final double T_PARAMETRIC_DONT_SHOOT = 0.55;
    public static final long WAIT_INIT_SHOOTER = 5;

    public static final double UNSHORTCUT_LENGTH = 10;
    public static final double MIN_RANGE_SCALE_BRAKING_STRENGTH = 20;

    public static double getBrakingStrengthScaleFromRange(double range) {
        if (range < MIN_RANGE_SCALE_BRAKING_STRENGTH){
            return BRAKING_STRENGTH_PEDRO_DINITECH;
        } else {
            return BRAKING_STRENGTH_PEDRO_DINITECH * MIN_RANGE_SCALE_BRAKING_STRENGTH / range;
        }
    }

    public static final double MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME = 37.0;
    public static double getLinearInterpolationHeadingEndTimeFromRange(double range){
        if (range > MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME){
            return LINEAR_HEADING_INTERPOLATION_END_TIME;
        } else {
            return LINEAR_HEADING_INTERPOLATION_END_TIME * range / MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME;
        }
    }


    public static final double MAX_POWER_ROW_PICK_ARTEFACTS = 0.23;
    public static final double GATEPICK_POWER = MAX_POWER_ROW_PICK_ARTEFACTS;
    public static final int MODE_RAMASSAGE_TELE_TIMEOUT = 300;
    public static final int MODE_RAMASSAGE_AUTO_TIMEOUT = 23;
    public static final Pose END_GAME_RED_POSE = new Pose(38.5, 33.5, 0);
    public static final Pose END_GAME_BLUE_POSE = END_GAME_RED_POSE.mirror(FIELD_SIDE_LENGTH);


    public static final Pose FIELD_CENTER_90HEADING_POSE = new Pose(72, 72, Math.PI/2);
    public static final Pose RESET_POSE_RED = new Pose(38.6, 33.4, 0);
    public static final Pose RESET_POSE_BLUE = RESET_POSE_RED.mirror(144);
    public static final Pose ROTATED_RESET_POSE_BLUE = RESET_POSE_BLUE.rotate(BLUE_TEAM_HEADING, true);



    //BLUE SIDE
    public static final Pose BLUE_BASKET_POSE = new Pose(8, 136, 0);
    public static final Pose ROTATED_BLUE_BASKET_POSE = BLUE_BASKET_POSE.rotate(BLUE_TEAM_HEADING, true);
    public static final Pose BLUE_AUDIENCE_POSE = new Pose(57, 9.3, Math.PI/2);
    public static final Pose BLUE_AUDIENCE_SHOOT_POSE = new Pose(58, 23, Math.toRadians(113));
    public static final Pose BLUE_GOAL_POSE = new Pose(21.9, 121.1,3 * Math.PI / 4);
    public static final Pose BLUE_RAMP_POSE = new Pose(16.5, 62.2, 0); // heading = -0.162
    public static double GATEPICK_LENGTH_BACKUP_X = -2.2;
    public static double GATEPICK_LENGTH_BACKUP_Y = -2.9;
    public static final Pose BLUE_RAMP_END_POSE = new Pose(BLUE_RAMP_POSE.getX() + GATEPICK_LENGTH_BACKUP_X, BLUE_RAMP_POSE.getY() + GATEPICK_LENGTH_BACKUP_Y, -0.71);
    public static final Pose BLUE_OPEN_RAMP_PICK_POSE = new Pose(14, 58.5, -0.75);
    public static final Pose OBELISK_BLUE_POSE = new Pose(61.4, 82.1, Math.PI/2.1);
    public static final Pose CLOSE_SHOOT_BLUE_POSE = new Pose(48.3, 95, Math.toRadians(131));
    public static final Pose LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE = new Pose(55, 85, Math.toRadians(79));
    public static final double CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY = linearSpeedFromPedroRange(CLOSE_SHOOT_BLUE_POSE.distanceFrom(BLUE_BASKET_POSE));
    public static final double AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY = linearSpeedFromPedroRange(BLUE_AUDIENCE_SHOOT_POSE.distanceFrom(BLUE_BASKET_POSE));
    public static final double INIT_SHOOT_AUTO_SHOOTER_VELOCITY = 3;
    public static final Pose FIRST_ROW_BLUE_POSE = new Pose(43, 83.5, 0);
    public static final Pose SECOND_ROW_BLUE_POSE = FIRST_ROW_BLUE_POSE
            .withY(FIRST_ROW_BLUE_POSE.getY() - TILE_DIM);
    public static final Pose THIRD_ROW_BLUE_POSE = SECOND_ROW_BLUE_POSE
            .withY(FIRST_ROW_BLUE_POSE.getY() - TILE_DIM);
    public static final Pose BLUE_VOID_POSE = new Pose(50, 62, Math.PI);
    public static final Pose BLUE_WALL_PICK_POSE = new Pose(14.5, 20, Math.toRadians(-55));




    //RED SIDE
    public static final Pose RED_BASKET_POSE = BLUE_BASKET_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose RED_AUDIENCE_POSE = BLUE_AUDIENCE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_AUDIENCE_SHOOT_POSE = BLUE_AUDIENCE_SHOOT_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose RED_GOAL_POSE = BLUE_GOAL_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_RAMP_POSE = BLUE_RAMP_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_RAMP_END_POSE = BLUE_RAMP_END_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_OPEN_RAMP_PICK_POSE = BLUE_OPEN_RAMP_PICK_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose OBELISK_RED_POSE = OBELISK_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose CLOSE_SHOOT_RED_POSE = CLOSE_SHOOT_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose LOOK_MOTIF_CLOSE_SHOOT_RED_POSE = LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose FIRST_ROW_RED_POSE = FIRST_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose SECOND_ROW_RED_POSE = SECOND_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose THIRD_ROW_RED_POSE = THIRD_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose RED_VOID_POSE = BLUE_VOID_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_WALL_PICK_POSE = BLUE_WALL_PICK_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final double LENGTH_WALL_PICK = 20;



    /**
     * Gamepads
     */
    public static final double RUMBLE_POWER = 1;
    public static final int RUMBLE_DURATION_1 = 100;
    public static final int RUMBLE_DURATION_2 = 200;
    public static final int RUMBLE_DURATION_3 = 20;
    public static final int RUMBLE_DURATION_4 = 40;




    /**
     *********************** SUBSYTEMS
     */
        /**
         ******** Mecanum Drive
         */
        /**
         * Constants
         */
        public static final double TELE_DRIVE_POWER = 0.3;
        public static final double TELE_DRIVE_POWER_TRIGGER_SCALE = 1 - TELE_DRIVE_POWER;
        public static final double SLOW_DRIVE_SCALE = 1;
        public static final double THROUGH_BORE_ENCODER_COUNTS_PER_REV = 8192; // https://revrobotics.eu/rev-11-1271/
        public static final double DEAD_WHEEL_DIAMETER_MM = 50.8; // https://revrobotics.eu/ION-Omni-Wheels/
        public static final double ENCODER_RESOLUTION = THROUGH_BORE_ENCODER_COUNTS_PER_REV / (DEAD_WHEEL_DIAMETER_MM * Math.PI);
        public static final double PAR_POD_Y_MM = -142.2; // -144.5;
        public static final double PERP_POD_X_MM = 143.2; //147;//143;
        public static final double CLAMPING_HEADING_ERROR = 0.39;
        public static final int NUMBER_CUSTOM_POWER_FUNC_DRIVE_PEDRO_LOCKED = 3;




    /**
         ******** Trieur
         */
        public static final String TRAPPE_SERVO_NAME = "porte";
        public static final double TRAPPE_OPEN_POSITION = 0;
        public static final double TRAPPE_CLOSE_POSITION = -130;
        public static final double TRAPPE_TELE_INCREMENT = 0.5;
        public static final long TRAPPE_OPEN_TIME = 470;
        public static final long TRAPPE_CLOSE_TIME = TRAPPE_OPEN_TIME;

        public static final String MOULIN_MOTOR_NAME = "moulin";

        public static int REVOLUTION_MOULIN_TICKS = 1833;
        public static double INTERVALLE_TICKS_MOULIN_DOUBLE = (double) REVOLUTION_MOULIN_TICKS / Moulin.TOTAL_POSITIONS; // = 305.5
        public static final double TICKS_TO_DEGREE = (double) 360 / REVOLUTION_MOULIN_TICKS; // 1 tick = 0.1964°
        // 1° = 5.1 ticks
        public static double getDegreesFromTicks(int ticks){
            return TICKS_TO_DEGREE * ticks;}
        public static int getTicksFromDegrees(double degrees){
            return (int) (degrees / TICKS_TO_DEGREE);}




        public static final double POWER_MOULIN_ROTATION = 1;
        public static final double POWER_MOULIN_ROTATION_OVERCURRENT = 0.5;
        public static int MOULIN_POSITION_TOLERANCE = getTicksFromDegrees(1.1);
        public static final int MOULIN_SPEED_TOLERANCE = 3; //10;
        public static double VERY_LOOSE_DEGREES = 2.5;

        public static final int MOULIN_POSITION_VERY_LOOSE_TOLERANCE = getTicksFromDegrees(VERY_LOOSE_DEGREES);

        public static final int MOULIN_ROTATE_SPEED_CONTINUOUS = 15 * (MOULIN_POSITION_TOLERANCE + 2);
        public static int MOULIN_ROTATE_SPEED_CALIBRATION = 18;
        public static final double SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF = 1;
        public static int WAIT_HIGH_SPEED_TRIEUR = 185;
        public static final double DISTANCE_ARTEFACT_IN_TRIEUR = 3.9;
        public static final double DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR = 1.1;
        public static final int OVER_CURRENT_BACKOFF_TICKS = 80; // Ticks to back off when over-current detected
        public static long WAIT_FOR_3BALL = 3800;

        //PIDF MOULIN (TURRET)
        public static double P_MOULIN_AGGRESSIVE = 5.954;// 6.54
        public static double I_MOULIN_AGGRESSIVE = 8.418;//11.03
        public static double D_MOULIN_AGGRESSIVE = 3.4;//0.7
        public static double F_MOULIN_AGGRESSIVE = 0.0;//1.493
        public static final double ADJUST_CONSTANT = 0.015;

        /**
         * *******Shooter
         */
        public static final String SHOOTER_MOTOR_NAME = "shooter";
        public static final int RUNNING_AVERAGE_SHOOTER_CURRENT_SIZE = 5;
        public static final int CURRENT_SHOOT_OVERFLOW = 1500;
        public static final double MAX_SHOOT_SPEED = 2800; // Ticks per second.
        public static final double SPEED_MARGIN = 15;
        public static double SPEED_MARGIN_SUPER_INTEL =  SPEED_MARGIN * 3;
        public static final double SPEED_INCREMENT_SHOOTER = 10;
        public static final double MAX_RANGE_TO_SHOOT_CM = 345;
        public static final double MIN_RANGE_TO_SHOOT_CM = 97;

        public static final double DIFF_MIN_MAX_RANGE_TO_SHOOT = MAX_RANGE_TO_SHOOT_CM - MIN_RANGE_TO_SHOOT_CM; // = 248;
        public static final double TELE_SHOOTER_SCALER = 30;
        public static final double SPEED_MARGIN_VISION_SHOOT = SPEED_MARGIN;

        public static final double P_SHOOTER_VELOCITY_AGGRESSIVE = 80;
        public static final double I_SHOOTER_VELOCITY_AGGRESSIVE = 1.33;
        public static final double D_SHOOTER_VELOCITY_AGGRESSIVE = 0.005;
        public static final double F_SHOOTER_VELOCITY_AGGRESSIVE = 2.1;

        public static double P_SHOOTER_VELOCITY_AGGRESSIVE_3R = P_SHOOTER_VELOCITY_AGGRESSIVE;
        public static double I_SHOOTER_VELOCITY_AGGRESSIVE_3R = 0.0001;
        public static double D_SHOOTER_VELOCITY_AGGRESSIVE_3R = 5.745;
        public static double F_SHOOTER_VELOCITY_AGGRESSIVE_3R = F_SHOOTER_VELOCITY_AGGRESSIVE * 5;
        public static final long SHOOT_REVOLUTION_THEN_WAIT = 300;
        public static final double MIN_RANGE_SHOOTER_SPEED = 1420;
        public static final double MAX_RANGE_SHOOTER_SPEED = 1970;

        public static final double DIFF_MIN_MAX_RANGE_SHOOTER_SPEED = MAX_RANGE_SHOOTER_SPEED - MIN_RANGE_SHOOTER_SPEED; // = 550;

        public static final double A_DIFFS = DIFF_MIN_MAX_RANGE_SHOOTER_SPEED / DIFF_MIN_MAX_RANGE_TO_SHOOT; // = 2.2177


    /**
     * Applies a linear function to get speed from range
     * y = a * m + b
     *
     * @param rangeCM The range value, positive.
     * @return The speed value, also positive
     */
    public static double linearSpeedFromRange(double rangeCM) {
        return A_DIFFS * rangeCM + 1204.879; //
    }



        /**
         * *******Chargeur
         */
        public static final String CHARGEUR_MOTOR_NAME = "chargeur";
        public static final String CHARGEUR_SERVO_GAUCHE_MOTOR_NAME = "chargeur_servo_gauche";
        public static final String CHARGEUR_SERVO_DROITE_MOTOR_NAME = "chargeur_servo_droite";

        public static final double ROULEAU_MOTOR_MAX_POWER = 0.85;
        public static final double TAPIS_MAX_SPEED = 0.99;
        public static final double CHARGEUR_INCREMENT = 0.1;

        /**
         ******** Sensors
         */
        public static final String CS1_NAME = "cs1";
        public static final String CS2_NAME = "cs2";
        public static final String CS3_NAME = "cs3";
        public static final String MAGNETIC_SWITCH_NAME = "m_s";
        public static final int MAGNETIC_ON_MOULIN_POSITION = 2;
        public static int OFFSET_MAGNETIC_POS = 41; // (int) Math.round(REVOLUTION_MOULIN_TICKS * 0.0250817);
        public static double SCALE_RECALIBRATION = getTicksFromDegrees(3);
        public static double POWER_SCALER_RECALIBRATION = 2; // = 15.2750000028

        public static final double DETECT_PURPLE_RED_RGB = 0.694;
        public static final double DETECT_PURPLE_GREEN_RGB = 0.612;
        public static final double DETECT_PURPLE_BLUE_RGB = 0.851;
        public static final double MARGIN_PURPLE_RGB_DETECTION = 0.25;

        public static final double DETECT_GREEN_RED_RGB = 0.2;
        public static final double DETECT_GREEN_GREEN_RGB = 0.9;
        public static final double DETECT_GREEN_BLUE_RGB = 0.2;
        public static final double MARGIN_GREEN_RGB_DETECTION = 0.35;

        public static final double GREEN_HUE_LOWER = 150;
        public static final double GREEN_HUE_HIGHER = 170;
        public static final double GREEN_SATURATION_LOWER = 0.58;
        public static final double GREEN_RED_RGB_HIGHER = 0.014;
        public static final double PURPLE_HUE_LOWER = 180;
        public static final double PURPLE_HUE_HIGHER = 260;
        public static final float GAIN_DETECTION = 20;
        public static final int SAMPLE_SIZE_TEST = 2;

        /**
         ******** Vision
         */
        public static final String CAMERA1_NAME = "CamColor";
        public static final double FX = 516.3798424;//0.0;// 516.3798424;//1;
                                            // https://github.com/jdhs-ftc/2025/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/atag/AprilTagLocalizer.kt
        public static final double FY = 515.8231389;//0.0;// 515.8231389; //1;
        public static final double CX = 328.1776587;// 0.0;// 328.1776587; //1;
        public static final double CY = 237.3745503;//0.0;// 237.3745503; //1;
        public static final double CAMERA_POSITION_X = -9.8;
        public static final double CAMERA_POSITION_Y = 2.8;
        public static final double CAMERA_POSITION_Z = 43.0;
        public static final double CAMERA_ORIENTATION_YAW = 0;
        public static final double CAMERA_ORIENTATION_PITCH = -90; // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
        public static final double CAMERA_ORIENTATION_ROLL = 0;
        // Choose a camera resolution. Not all cameras support all resolutions.
        public static final int CAMERA_WIDTH = 640;// 1280; // 640;
        public static final int CAMERA_HEIGHT = 480;// 800; // 480;
        public static final Size CAMERA_RESOLUTION = new Size(CAMERA_WIDTH, CAMERA_HEIGHT); // new Size(640, 480);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG; // Or YUY2

        public static final double CLAMP_BEARING = 73;
        public static final double MIN_RANGE_VISION = MIN_RANGE_TO_SHOOT_CM; //CM
        public static final double MAX_RANGE_VISION = MAX_RANGE_TO_SHOOT_CM; //CM

        public static final double DIFF_RANGE_VISION = MAX_RANGE_VISION - MIN_RANGE_VISION; // = 248;

        public static final double OFFSET_BEARING_AT_MIN_RANGE = -4.45; // DEGREES
        public static final double OFFSET_BEARING_AT_MAX_RANGE = -1.8; //DEGREES
        public static final double DIFF_OFFSET_BEARING_AT = OFFSET_BEARING_AT_MAX_RANGE - OFFSET_BEARING_AT_MIN_RANGE; // = -2.65
        public static final double CORRECTION_BASKET_OFFSET = 0.21;
        public static final double BASKET_Y_OFFSET = 8;
        public static final int NUMBER_AT_SAMPLES = 3;
        public static final int NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED = 4;
        public static final double MIN_LINEAR = 0.005;
        public static final double CUSTOM_POWER_LOCKED = 0.05;

        public static final double DIFF_A_BEARING = DIFF_OFFSET_BEARING_AT / DIFF_RANGE_VISION; // = -0.01068548387

        public static final double aAT_LINE = 1.4;
        public static final double bAT_LINE = 59;


        /**
         * Calculates linear interpolation bearing offset using linear interpolation based on range.
         * This helps correct for parallax error due to camera placement.
         * @param range The range to the target.
         * @return The calculated bearing offset.
         */
        public static double getLinearInterpolationOffsetBearing(double range) {
            return DIFF_A_BEARING * range -5.4865;
        }

        public static double pickCustomPowerFunc(double x, int funcNumber) {
                switch (funcNumber) {
                    case 2:
                            return customPowerFunc2(x);
                    case 3:
                            return customPowerFunc3(x);
                    case 4:
                            return customPowerFunc4(x);
                    case 5:
                            return customLinearFunc(x);
                    case 6:
                        return customPowerFunc5(x);
                    default:
                            return customPowerFunc(x);
                }
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.5;
                double shifted3 = shifted * shifted * shifted; // Faster than Math.pow(shifted, 3)

                return Math.signum(x) * (3 * shifted3 + absValue / 4 + 0.375);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc2(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.6;
                // Manually expand pow(shifted, 5) = shifted * shifted * shifted * shifted *
                // shifted
                double shifted2 = shifted * shifted;
                double shifted4 = shifted2 * shifted2;
                double shifted5 = shifted4 * shifted;

                return Math.signum(x) * (8.1644718783 * shifted5 + absValue / 3.551975 + 0.6538649554);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc3(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.65;
                double shifted2 = shifted * shifted;
                double shifted4 = shifted2 * shifted2;
                double shifted5 = shifted4 * shifted;

                return Math.signum(x)
                                * (7.6704545455 * shifted5 + absValue / 14.08 + 0.8920454545);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc4(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.56;
                double shifted3 = shifted * shifted * shifted;

                return Math.signum(x)
                                * (1.4705882353 * shifted3 + absValue * 0.5882352941 + 0.2647058824);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc5(double x) {
            double absValue = Math.abs(x);
            double shifted = absValue - 0.405;
            double shifted7 = shifted * shifted * shifted * shifted * shifted * shifted * shifted;

            return Math.signum(x)
                    * (224.22 * shifted7 + absValue * 0.07407407407 + 0.4192666667);
        }

        /**
         * Applies a custom Linear function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customLinearFunc(double x) {
                if (Math.abs(x) < MIN_LINEAR) {
                        return 0;
                }
                return Math.signum(x) * (x * (1 - MIN_LINEAR) + MIN_LINEAR);
        }

        public static double cmToInch(double cm){
            return cm * 0.3937007874;
        }
        public static double inchToCm(double inch){
                return inch * 2.54;
        }
        public static double mmToInch(double cm){
            return cmToInch(cm) * 0.1;
        }


    /**
     * A zero-allocation circular buffer for calculating a running average.
     */
    public static class RunningAverage {
        private final double[] samples;
        private int index = 0;
        private int count = 0;
        private double runningSum = 0;

        public RunningAverage(int size) {
            this.samples = new double[size];
        }

        public void add(double value) {
            runningSum -= samples[index];
            samples[index] = value;
            runningSum += value;
            index = (index + 1) % samples.length;
            if (count < samples.length) {
                count++;
            }
        }

        public Double getAverage() {
            return count == 0 ? null : runningSum / count;
        }

        public double getStd() {
            if (count == 0) return 0;
            double mean = getAverage();
            double sumSquaredDiffs = 0;
            for (int i = 0; i < count; i++) {
                double diff = samples[i] - mean;
                sumSquaredDiffs += diff * diff;
            }
            return Math.sqrt(sumSquaredDiffs / count);
        }

        public boolean isEmpty() {
            return count == 0;
        }

        public int size() {
            return count;
        }
    }
}
