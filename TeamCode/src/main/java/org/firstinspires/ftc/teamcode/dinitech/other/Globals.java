package org.firstinspires.ftc.teamcode.dinitech.other;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class Globals {
        /**
         * Tests variables
         */
        public static final double TEST_CIRCLE_RADIUS = 10.0;
        public static final double TEST_CONSTRAINTS = 1.0;

        // Pose2d from the BEGIN_POSE to create left, right, forward and backward circle
        public static final Pose2d LEFT_CIRCLE_CENTER_POSE = new Pose2d(new Vector2d(0, TEST_CIRCLE_RADIUS), 0);
        public static final Pose2d RIGHT_CIRCLE_CENTER_POSE = new Pose2d(new Vector2d(0, -TEST_CIRCLE_RADIUS), 0);
        public static final Pose2d FORWARD_CIRCLE_CENTER_POSE = new Pose2d(new Vector2d(TEST_CIRCLE_RADIUS, 0), 0);
        public static final Pose2d BACKWARD_CIRCLE_CENTER_POSE = new Pose2d(new Vector2d(-TEST_CIRCLE_RADIUS, 0), 0);

        /**
         * Auto Phase globals
         */
        public static final double TILE_DIM = 24;
        public static final double HYPOTHENUSE_GOAL = 26.5;
        public static final double ANGLE_RADIANS_BLUE_GOAL = Math.acos(60.95 / 75.30); // 24 inches = 60.95 cm // 26.5 inches = 75.30 cm
        public static final double ANGLE_RADIANS_RED_GOAL = - ANGLE_RADIANS_BLUE_GOAL;

        public static final double LENGTH_ARTEFACT_ROW = TILE_DIM * 0.9;
        public static final double DISTANCE_BETWEEN_ARTEFACT_ROW = TILE_DIM * 1;

        public static final Pose2d BLUE_GOAL_BEGIN_POSE = new Pose2d(-2 * TILE_DIM, -2 * TILE_DIM, Math.PI / 4);
        public static final Pose2d OBELISK_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, Math.PI);
        public static final Pose2d CLOSE_SHOOT_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, 1.25 * Math.PI);

        public static final Pose2d FIRST_ROW_ARTEFACTS_PREP_POSE = new Pose2d(-TILE_DIM / 2, -1.1 * TILE_DIM, Math.PI / 2);
        public static final Pose2d SECOND_ROW_ARTEFACTS_PREP_POSE = new Pose2d(
                        new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW,
                                        FIRST_ROW_ARTEFACTS_PREP_POSE.position.y),
                        FIRST_ROW_ARTEFACTS_PREP_POSE.heading);
        public static final Pose2d THIRD_ROW_ARTEFACTS_PREP_POSE = new Pose2d(
                        new Vector2d(SECOND_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW,
                                        SECOND_ROW_ARTEFACTS_PREP_POSE.position.y),
                        SECOND_ROW_ARTEFACTS_PREP_POSE.heading);

        public static final Pose2d FULL_BASE_POSE = new Pose2d(1.55 * TILE_DIM, -1.35 * TILE_DIM, Math.PI);

        /**
         * Gamepads
         */
        public static final double RUMBLE_POWER = 1;

        /**
         *********************** SUBSYTEMS
         */
        /**
         ******** Mecanum Drive
         */
        /**
         * Constants
         */
        public static final Pose2d BEGIN_POSE = new Pose2d(0, 0, 0);
        public static final double TELE_DRIVE_POWER = 0.3;
        public static final double TELE_DRIVE_POWER_TRIGGER_SCALE = 1 - TELE_DRIVE_POWER;
        public static final int DRIVER_POWER_SCALER_TO_THE_POWER = 3;
        public static final double SLOW_DRIVE_SCALE = 0.3;

        public static final double MAX_WHEEL_VELOCITY = 50.0;
        public static final double MIN_PROFILE_ACCELERATION = -30;
        public static final double MAX_PROFILE_ACCELERATION = 50;
        public static final double MAX_ANGLE_VELOCITY = Math.PI;
        public static final double MAX_ANGLE_ACCELERATION = Math.PI;
        public static final double AXIAL_GAIN = 0.2;
        public static final double LATERAL_GAIN = 0.2;
        public static final double HEADING_GAIN = 0.3;
        public static final double AXIAL_VELOCITY_GAIN = 0.01 * AXIAL_GAIN;
        public static final double LATERAL_VELOCITY_GAIN = 0.01 * LATERAL_GAIN;
        public static final double HEADING_VELOCITY_GAIN = 0.01 * HEADING_GAIN;

        /**
         ******** Trieur
         */
        public static final String TRAPPE_SERVO_NAME = "porte";
        public static final double TRAPPE_OPEN_POSITION = 0;
        public static final double TRAPPE_CLOSE_POSITION = -100;
        public static final double TRAPPE_TELE_INCREMENT = 0.5;
        public static final String MOULIN_MOTOR_NAME = "moulin";

        public static final int MOTOR_TICKS_PER_REV = 288;
        public static final int RAPPORT_TRANSMISSION = 8; // pignon 18 dents ; courronne 144 dents // rapport de
                                                          // transmission : 144/8 = 8;
        public static int INTERVALLE_TICKS_MOULIN = 240;//384;// MOTOR_TICKS_PER_REV * RAPPORT_TRANSMISSION /
                                                              // Moulin.TOTAL_POSITIONS;
        public static final double POWER_MOULIN_ROTATION = 1;
    public static double POWER_MOULIN_CALIBRATION_ROTATION = POWER_MOULIN_ROTATION;

    public static final int MOULIN_POSITION_TOLERANCE = 2;

        public static final int MOULIN_ROTATE_SPEED_CONTINUOUS = 6 * MOULIN_POSITION_TOLERANCE;
        public static int MOULIN_ROTATE_SPEED_CALIBRATION = 4;
        public static final int TRIEUR_TIME_BETWEEN_SHOTS = 5; // milliseconds
        public static final double DISTANCE_ARTEFACT_IN_TRIEUR = 3.5;
        public static final double DISTANCE_MARGIN_ARTEFACT_IN_TRIEUR = 1.8;
        public static final int OVER_CURRENT_BACKOFF_TICKS = 20; // Ticks to back off when over-current detected

        //PIDF MOULIN (TURRET)
        public static final double P_MOULIN_AGGRESSIVE = 2.231813;
        public static final double I_MOULIN_AGGRESSIVE = 11.941806;
        public static final double D_MOULIN_AGGRESSIVE = 0.008558;
        public static final double F_MOULIN_AGGRESSIVE = 0.178335;
        public static final double ADJUST_CONSTANT = 0.0005;

        //FF COEF
        public static final double kS_MOULIN_AGGRESSIVE = 0;
        public static final double kV_MOULIN_AGGRESSIVE = 0;
        public static final double kA_MOULIN_AGGRESSIVE = 0;

        /**
         * *******Shooter
         */
        public static final String SHOOTER_MOTOR_NAME = "shooter";
        public static final double MAX_SHOOT_SPEED = 2540; // Ticks per second.
        public static final double SPEED_MARGIN = 10;
        public static final double SPEED_INCREMENT_SHOOTER = 10;
        public static final double MAX_RANGE_TO_SHOOT = 120;
        public static final double MIN_RANGE_TO_SHOOT = 30;
        public static final double TELE_SHOOTER_SCALER = 20;
        public static final double SPEED_MARGIN_VISION_SHOOT = SPEED_MARGIN;
        public static final double ACCELERATION_SCALE_SHOOTER = 100;
        public static final double P_SHOOTER_VELOCITY_AGGRESSIVE = 2.807486;
        public static final double I_SHOOTER_VELOCITY_AGGRESSIVE = 0;
        public static final double D_SHOOTER_VELOCITY_AGGRESSIVE = 0;
        public static final double F_SHOOTER_VELOCITY_AGGRESSIVE = 13.272119;

        /**
         * *******Chargeur
         */
        public static final String CHARGEUR_MOTOR_NAME = "chargeur";
        public static final String CHARGEUR_SERVO_MOTOR_NAME = "chargeur_servo";
        public static final double CHARGEUR_MOTOR_POWER = 1;
        public static final double CHARGEUR_SPEED = 2300;
        public static final double CHARGEUR_INCREMENT = 0.1;

        /**
         ******** Sensors
         */
        public static final String CS1_NAME = "cs1";
        public static final String CS2_NAME = "cs2";
        public static final String CS3_NAME = "cs3";
        public static final String CS4_NAME = "cs4";
        public static final String MAGNETIC_SWITCH_NAME = "m_s";
        public static final int MAGNETIC_ON_MOULIN_POSITION = 2;
        public static int OFFSET_MAGNETIC_POS = 12;
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
        public static double FX = 516.3798424;//0.0;// 516.3798424;//1;
                                            // https://github.com/jdhs-ftc/2025/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/atag/AprilTagLocalizer.kt
        public static double FY = 515.8231389;//0.0;// 515.8231389; //1;
        public static double CX = 328.1776587;// 0.0;// 328.1776587; //1;
        public static double CY = 237.3745503;//0.0;// 237.3745503; //1;
        public static double CAMERA_POSITION_X = -9.0;
        public static double CAMERA_POSITION_Y = 0.0;
        public static double CAMERA_POSITION_Z = 43.0;

        public static double OFFSET_ROBOT_X = 16.0;
        public static double OFFSET_ROBOT_Y = 16.5;
        public static double OFFSET_ROBOT_YAW = Math.PI + ANGLE_RADIANS_BLUE_GOAL;

        public static final double CAMERA_ORIENTATION_YAW = 0;
        public static final double CAMERA_ORIENTATION_PITCH = -90; // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
        public static final double CAMERA_ORIENTATION_ROLL = 0;
        public static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
        // Choose a camera resolution. Not all cameras support all resolutions.
        public static final int CAMERA_WIDTH = 640;// 1280; // 640;
        public static final int CAMERA_HEIGHT = 480;// 800; // 480;
        public static final Size CAMERA_RESOLUTION = new Size(CAMERA_WIDTH, CAMERA_HEIGHT); // new Size(640, 480);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG; // Or YUY2

        public static double CLAMP_BEARING = 62;
        public static final double MIN_RANGE_VISION = 40; //INCHES
    public static final double MAX_RANGE_VISION = 140; //INCHES
    public static final double DIFFERENCE_RANGE_VISION = MAX_RANGE_VISION - MIN_RANGE_VISION;

    public static final double OFFSET_BEARING_AT_40_INCHES_RANGE = -7.18; // DEGREES
        public static final double OFFSET_BEARING_AT_140_INCHES_RANGE = -1.47; //DEGREES
        public static double SCALER_OFFSET_AT_TO_X_BASKET = 0.000001;
        public static double BASKET_Y_OFFSET = 8;
        public static int NUMBER_AT_SAMPLES = 3;
        public static int NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED = 1;
        public static double MIN_LINEAR = 0.005;
        public static double CUSTOM_POWER_LOCKED = 0.05;

        public static double pickCustomPowerFunc(double x, int funcNumber) {
                switch (funcNumber) {
                        case 1:
                                return customPowerFunc(x);
                        case 2:
                                return customPowerFunc2(x);
                        case 3:
                                return customPowerFunc3(x);
                        case 4:
                                return customPowerFunc4(x);
                        case 5:
                                return customLinearFunc(x);
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

}
