package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoPathsDefinitions {
    public static double LINEAR_HEADING_INTERPOLATION_END_TIME = 0.8;
    public static final double LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT = 0.55;


    public static double FOLLOWER_T_POSITION_END = 0.97;//0.91;
    public static double FOLLOWER_T_POSITION_END_TELEOP = 0.985;
    public static double LINEAR_B_FOLLOWER_T_END = 0.81;
    public static double LINEAR_A_FOLLOWER_T_END = 0.0021;
    public static final double LENGTH_X_ROW = TILE_DIM * 0.97;
    public static final double LENGTH_X_ROW_3RD = TILE_DIM * 0.98;
    public static final double T_PARAMETRIC_DONT_SHOOT = 0.55;
    public static long WAIT_INIT_PEDRO_SHOOTER = 70;
    public static final double UNSHORTCUT_LENGTH = 10;
    public static final double MIN_RANGE_SCALE_BRAKING_STRENGTH = 30.0;

    public static double RADIUS_RAMP_PICK = TILE_DIM/8;//TILE_DIM/4;
    public static long TIME_AT_GATE_GATE_PICK = 250;
    public static double GATE_UNSHORTCUT_SCALE = 1.6;

    public static double getBrakingStrengthScaleFromRange(double range) {
        if (range < MIN_RANGE_SCALE_BRAKING_STRENGTH){
            return 0.6;
        } else {
            return 0.6 * Math.pow(MIN_RANGE_SCALE_BRAKING_STRENGTH / range, 2);
        }
    }

    public static double MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME = 25.0;
    public static double getLinearInterpolationHeadingEndTimeFromRange(double range){
        if (range >= MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME){
            return LINEAR_HEADING_INTERPOLATION_END_TIME;
        } else {
            return LINEAR_HEADING_INTERPOLATION_END_TIME * range / MAX_RANGE_SCALE_LINEAR_INTERPOLATION_END_TIME;
        }
    }


    public static double MAX_POWER_ROW_PICK_ARTEFACTS = 0.85;
    public static final double GATEPICK_POWER = 1;

    public static final double SCALER_TO_PICK_POSE = 0.65;
    public static double getPedroFieldFromUnitNormalized(double unitNormalized){
        return (unitNormalized + 1) * FieldDefinitions.FIELD_SIDE_LENGTH/2;
    }

    public static Pose getPedroPoseFromUnitNormalized(double x, double y, double heading){
        return new Pose(getPedroFieldFromUnitNormalized(x), getPedroFieldFromUnitNormalized(y), heading);
    }

    public static double getDynamicTEndFollower(double distance){
        if (distance < 10) return 0.011*distance + 0.7;

        double linear = LINEAR_A_FOLLOWER_T_END * distance + LINEAR_B_FOLLOWER_T_END;

        return Math.max(Math.min(linear, FOLLOWER_T_POSITION_END), LINEAR_B_FOLLOWER_T_END);
    }
}
