package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.BRAKING_STRENGTH_PEDRO_DINITECH;

public class AutoPathsDefinitions {
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME = 0.81;
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT = 0.55;


    public static final double FOLLOWER_T_POSITION_END = 0.9;//0.91;
    public static double FOLLOWER_T_POSITION_END_TELEOP = 0.96;
    public static final double LENGTH_X_ROW = TILE_DIM * 0.86;
    public static final double LENGTH_X_ROW_3RD = TILE_DIM * 1;

    public static final double T_PARAMETRIC_DONT_SHOOT = 0.55;
    public static final long WAIT_INIT_SHOOTER = 5;
    public static final long WAIT_INIT_PEDRO_SHOOTER = 180;

    public static final double UNSHORTCUT_LENGTH = 10;
    public static final double MIN_RANGE_SCALE_BRAKING_STRENGTH = 30.0;

    public static double getBrakingStrengthScaleFromRange(double range) {
        if (range < MIN_RANGE_SCALE_BRAKING_STRENGTH){
            return BRAKING_STRENGTH_PEDRO_DINITECH;
        } else {
            return BRAKING_STRENGTH_PEDRO_DINITECH * Math.pow(MIN_RANGE_SCALE_BRAKING_STRENGTH / range, 2);
        }
    }

    public static final double MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME = 40.0;
    public static double getLinearInterpolationHeadingEndTimeFromRange(double range){
        if (range > MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME){
            return LINEAR_HEADING_INTERPOLATION_END_TIME;
        } else {
            return LINEAR_HEADING_INTERPOLATION_END_TIME * range / MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME;
        }
    }


    public static final double MAX_POWER_ROW_PICK_ARTEFACTS = 0.23;
    public static final double GATEPICK_POWER = MAX_POWER_ROW_PICK_ARTEFACTS;
}
