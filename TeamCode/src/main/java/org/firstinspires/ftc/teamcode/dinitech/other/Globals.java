package org.firstinspires.ftc.teamcode.dinitech.other;


import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
public class Globals {
    public static long TELEMETRY_UPDATE_INTERVAL_MS = 500;

    /**
     * Auto Phase globals
     */
    public static final double AUTO_ROBOT_CONSTRAINTS = 1;
    public static final double BRAKING_STRENGTH_PEDRO_DINITECH = 1.4;
    public static final double BRAKING_START_PEDRO_DINITECH = 1.4;
    public static double LINEAR_HEADING_INTERPOLATION_END_TIME = 0.93;
    public static final double TILE_DIM = 24;
    public static final double SCALE_Y_TILE = 1.05;
    public static double FOLLOWER_T_POSITION_END = 0.885;//0.93;
    public static double LENGTH_X_ROW = TILE_DIM * 0.85;
    public static double LENGTH_X_ROW_SUPER = 25;
    public static double LENGTH_X_ROW_SUPER_23RD = 32.5;


    public static final double MAX_POWER_ROW_PICK_ARTEFACTS = 0.23;
    public static double GATEPICK_POWER = 0.18;
    public static double SUPER_POWER_ROW_PICK_ARTEFACTS = 1;
    public static final int MODE_RAMASSAGE_TELE_TIMEOUT = 300;
    public static final int MODE_RAMASSAGE_AUTO_TIMEOUT = 50;
    public static long WAIT_AT_END_ROW = 200;
    public static final Pose END_GAME_RED_POSE = new Pose(38.5, 33.5, 0);
    public static final Pose END_GAME_BLUE_POSE = END_GAME_RED_POSE.mirror();


    public static final Pose FIELD_CENTER_90HEAING_POSE = new Pose(72, 72, Math.PI/2);



    //BLUE SIDE
    public static final Pose BLUE_BASKET_POSE = new Pose(11, 137, 0);
    public static final Pose BLUE_SMALL_TRIANGLE_POSE = new Pose(57, 9.3, Math.PI/2);
    public static final Pose BLUE_SMALL_TRIANGLE_SHOOT_POSE = new Pose(60, 20, Math.toRadians(115));
    public static final Pose BLUE_GOAL_POSE = new Pose(22, 121, (double) 7 / 4 * Math.PI);
    public static  final Pose BLUE_RAMP_POSE = new Pose(20, 62, 0);
    public static final double GATEPICK_LENGTH_BACKUP_X = 7;
    public static final double GATEPICK_LENGTH_BACKUP_Y = 1;
    public static  final Pose BLUE_RAMP_END_POSE = new Pose(BLUE_RAMP_POSE.getX() - GATEPICK_LENGTH_BACKUP_X, BLUE_RAMP_POSE.getY() - GATEPICK_LENGTH_BACKUP_Y, Math.toRadians(-28));

    public static final Pose OBELISK_BLUE_POSE = new Pose(61.4, 82.1, Math.PI/2.1);

    public static final Pose CLOSE_SHOOT_BLUE_POSE = new Pose(48.3, 95, 2.925*Math.PI/4);
    public static final double CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY = 1405;
    public static final double SMALL_TRIANGLE_AUTO_SHOOTER_VELOCITY = 1730;
    public static final Pose FIRST_ROW_BLUE_POSE = new Pose(43, 84, 0);
    public static final Pose SECOND_ROW_BLUE_POSE = FIRST_ROW_BLUE_POSE.withY(FIRST_ROW_BLUE_POSE.getY()-TILE_DIM*SCALE_Y_TILE);
    public static final Pose THIRD_ROW_BLUE_POSE = SECOND_ROW_BLUE_POSE.withY(SECOND_ROW_BLUE_POSE.getY()-TILE_DIM*SCALE_Y_TILE);



    //RED SIDE
    public static final Pose RED_BASKET_POSE = BLUE_BASKET_POSE.mirror();

    public static final Pose RED_SMALL_TRIANGLE_POSE = BLUE_SMALL_TRIANGLE_POSE.mirror();
    public static final Pose RED_SMALL_TRIANGLE_SHOOT_POSE = BLUE_SMALL_TRIANGLE_SHOOT_POSE.mirror();

    public static final Pose RED_GOAL_POSE = BLUE_GOAL_POSE.mirror();
    public static final Pose RED_RAMP_POSE = BLUE_RAMP_POSE.mirror();
    public static final Pose RED_RAMP_END_POSE = BLUE_RAMP_END_POSE.mirror();

    public static final Pose OBELISK_RED_POSE = OBELISK_BLUE_POSE.mirror();

    public static final Pose CLOSE_SHOOT_RED_POSE = CLOSE_SHOOT_BLUE_POSE.mirror();
    public static final Pose FIRST_ROW_RED_POSE = FIRST_ROW_BLUE_POSE.mirror();
    public static final Pose SECOND_ROW_RED_POSE = SECOND_ROW_BLUE_POSE.mirror();
    public static final Pose THIRD_ROW_RED_POSE = THIRD_ROW_BLUE_POSE.mirror();

    /**
     * Gamepads
     */
    public static final double RUMBLE_POWER = 1;
    public static final int RUMBLE_DURATION_1 = 100;
    public static final int RUMBLE_DURATION_2 = 200;
    public static int RUMBLE_DURATION_3 = 20;
    public static int RUMBLE_DURATION_4 = 40;




    /**
     *********************** SUBSYTEMS
     */
        /**
         ******** Mecanum Drive
         */
        /**
         * Constants
         */
        public static final Pose BEGIN_POSE = new Pose(0, 0, 0);
        public static final double TELE_DRIVE_POWER = 0.3;
        public static final double TELE_DRIVE_POWER_TRIGGER_SCALE = 1 - TELE_DRIVE_POWER;
        public static final int DRIVER_POWER_SCALER_TO_THE_POWER = 3;
        public static final double SLOW_DRIVE_SCALE = 1;
        public static final double THROUGH_BORE_ENCODER_COUNTS_PER_REV = 8192; // https://revrobotics.eu/rev-11-1271/
        public static final double DEAD_WHEEL_DIAMETER_MM = 50.8; // https://revrobotics.eu/ION-Omni-Wheels/
        public static final double ENCODER_RESOLUTION = THROUGH_BORE_ENCODER_COUNTS_PER_REV / (DEAD_WHEEL_DIAMETER_MM * Math.PI);
        public static final double PAR_POD_Y_MM = -127;
        public static final double PERP_POD_X_MM = 143;
        public static double CORRECTION_FENCE_PEDRO_AIM = 0.00001;
        public static double PEDRO_AIMING_CONTROLLER_P =0.0001;
        public static double PEDRO_AIMING_CONTROLLER_I =0.0001;
        public static double PEDRO_AIMING_CONTROLLER_D =0.0001;
        public static double PEDRO_AIMING_CONTROLLER_F =0.0001;




    /**
         ******** Trieur
         */
        public static int MOULIN_TICKS_TO_WAIT_DOUBLE_SERVO = 10;
        public static final String TRAPPE_SERVO_NAME = "porte";
        public static final double TRAPPE_OPEN_POSITION = 0;
        public static final double TRAPPE_CLOSE_POSITION = -130;
        public static final double TRAPPE_TELE_INCREMENT = 0.5;
        public static final long TRAPPE_OPEN_TIME = 450;
        public static final long TRAPPE_CLOSE_TIME = TRAPPE_OPEN_TIME;

        public static final String MOULIN_MOTOR_NAME = "moulin";

        public static final int MOTOR_TICKS_PER_REV = 288;
        public static final int RAPPORT_TRANSMISSION = 8; // pignon 18 dents ; courronne 144 dents // rapport de
                                                          // transmission : 144/8 = 8;
        public static int INTERVALLE_TICKS_MOULIN = 193; //240;//384;// MOTOR_TICKS_PER_REV * RAPPORT_TRANSMISSION /
                                                              // Moulin.TOTAL_POSITIONS;
        public static final double POWER_MOULIN_ROTATION = 1;
        public static final double POWER_MOULIN_ROTATION_OVERCURRENT = 0.5;
        public static final double POWER_MOULIN_CALIBRATION_ROTATION = POWER_MOULIN_ROTATION;

        public static int MOULIN_POSITION_TOLERANCE = 0; // 2;
        public static int SCALE_MOULIN_POSITION_TOLERANCE_LOOSE = 3;
        public static final int MOULIN_SPEED_TOLERANCE = 3; //10;
        public static int SCALE_MOULIN_SPEED_TOLERANCE_LOOSE = 3;
        public static final int MOULIN_POSITION_LOOSE_TOLERANCE = (MOULIN_POSITION_TOLERANCE +1) * SCALE_MOULIN_POSITION_TOLERANCE_LOOSE;
        public static final int MOULIN_SPEED_LOOSE_TOLERANCE = MOULIN_SPEED_TOLERANCE * SCALE_MOULIN_POSITION_TOLERANCE_LOOSE;
        public static int SCALE_MOULIN_SPEED_TOLERANCE_VERY_LOOSE = 6;

        public static final int MOULIN_POSITION_VERY_LOOSE_TOLERANCE = (MOULIN_POSITION_TOLERANCE+1) * SCALE_MOULIN_SPEED_TOLERANCE_VERY_LOOSE;
        public static final int MOULIN_SPEED_VERY_LOOSE_TOLERANCE = MOULIN_SPEED_TOLERANCE * SCALE_MOULIN_SPEED_TOLERANCE_VERY_LOOSE;

        public static final int MOULIN_ROTATE_SPEED_CONTINUOUS = 6 * (MOULIN_POSITION_TOLERANCE+1);
        public static int MOULIN_ROTATE_SPEED_CALIBRATION = 10;
        public static final double SCALE_DISTANCE_ARTEFACT_IN_TRIEUR_COEF = 1.4;

        public static final double DISTANCE_ARTEFACT_IN_TRIEUR = 3.5;
        public static final double DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR = 1.8;
        public static final int OVER_CURRENT_BACKOFF_TICKS = 80; // Ticks to back off when over-current detected

        //PIDF MOULIN (TURRET)
        public static double P_MOULIN_AGGRESSIVE = 10;
        public static double I_MOULIN_AGGRESSIVE = 11.8;
        public static double D_MOULIN_AGGRESSIVE = 0.0;
        public static double F_MOULIN_AGGRESSIVE = 0.0;
        public static double ADJUST_CONSTANT = 0.005;

        /**
         * *******Shooter
         */
        public static final String SHOOTER_MOTOR_NAME = "shooter";
        public static final double MAX_SHOOT_SPEED = 2800; // Ticks per second.
        public static final double SPEED_MARGIN = 15;
        public static final double SPEED_INCREMENT_SHOOTER = 10;
        public static final double MAX_RANGE_TO_SHOOT_CM = 345;
        public static final double MIN_RANGE_TO_SHOOT_CM = 97;

        public static final double DIFF_MIN_MAX_RANGE_TO_SHOOT = MAX_RANGE_TO_SHOOT_CM - MIN_RANGE_TO_SHOOT_CM; // = 248;
        public static final double TELE_SHOOTER_SCALER = 20;
        public static final double SPEED_MARGIN_VISION_SHOOT = SPEED_MARGIN;
        public static final double CLOSE_SHOOT_SHOOTER_VELOCITY = 1400;
        public static final double MID_SHOOT_SHOOTER_VELOCITY = 1600;
        public static final double LONG_SHOOT_SHOOTER_VELOCITY = 1850;
        public static final double P_SHOOTER_VELOCITY_AGGRESSIVE = 80;//2.807486;
        public static final double I_SHOOTER_VELOCITY_AGGRESSIVE = 1.33;
        public static final double D_SHOOTER_VELOCITY_AGGRESSIVE = 0;
        public static final double F_SHOOTER_VELOCITY_AGGRESSIVE = 2.1; //13.272119;

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
     * Gives back a linear speed from a range in inches
     * @param rangeInch The range value in inches, positive.
     * @return The speed value, also positive
     */
    public static double linearSpeedFromPedroRange(double rangeInch) {
        return 4.571 * rangeInch + 1148.145;
    }

        /**
         * *******Chargeur
         */
        public static final String CHARGEUR_MOTOR_NAME = "chargeur";
        public static final String CHARGEUR_SERVO_GAUCHE_MOTOR_NAME = "chargeur_servo_gauche";
        public static final String CHARGEUR_SERVO_DROITE_MOTOR_NAME = "chargeur_servo_droite";

        public static final double CHARGEUR_MOTOR_POWER = 1;
        public static final double SCALE_CHARGEUR_MOTOR_POWER = 0.4;
        public static final double CHARGEUR_SPEED = 2300;
        public static final double CHARGEUR_INCREMENT = 0.1;

        /**
         ******** Sensors
         */
        public static final String CS1_NAME = "cs1";
        public static final String CS2_NAME = "cs2";
        public static final String CS3_NAME = "cs3";
        public static final String MAGNETIC_SWITCH_NAME = "m_s";
        public static final int MAGNETIC_ON_MOULIN_POSITION = 2;
        public static int OFFSET_MAGNETIC_POS = 21;
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
        public static double CAMERA_POSITION_X = -9.8;
        public static double CAMERA_POSITION_Y = 2.8;
        public static double CAMERA_POSITION_Z = 43.0;
        public static double CAMERA_ORIENTATION_YAW = 0;
        public static double CAMERA_ORIENTATION_PITCH = -90; // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
        public static double CAMERA_ORIENTATION_ROLL = 0;
        // Choose a camera resolution. Not all cameras support all resolutions.
        public static int CAMERA_WIDTH = 1280;// 1280; // 640;
        public static int CAMERA_HEIGHT = 800;// 800; // 480;
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


        public static double VISION_RE_POSE_AT_RANGE = 50;


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

        public boolean isEmpty() {
            return count == 0;
        }

        public int size() {
            return count;
        }
    }
}
