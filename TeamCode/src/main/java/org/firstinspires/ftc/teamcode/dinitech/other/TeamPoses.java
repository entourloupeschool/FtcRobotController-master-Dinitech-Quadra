package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.FIELD_SIDE_LENGTH;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.linearSpeedFromPedroRange;

import com.pedropathing.geometry.Pose;

public class TeamPoses {
    public static double BLUE_TEAM_HEADING = Math.PI;
    public static final Pose OBELISK_POSE = new Pose(72, 145, Math.PI/2);
    public static final Pose END_GAME_RED_POSE = new Pose(38.5, 33.5, 0);
    public static final Pose END_GAME_BLUE_POSE = END_GAME_RED_POSE.mirror(FIELD_SIDE_LENGTH);


    public static final Pose RESET_POSE_RED = new Pose(38.6, 33.4, 0);
    public static final Pose RESET_POSE_BLUE = RESET_POSE_RED.mirror(FIELD_SIDE_LENGTH);
    public static final Pose ROTATED_RESET_POSE_BLUE = RESET_POSE_BLUE.rotate(BLUE_TEAM_HEADING, true);

    public static final Pose BLUE_BASKET_POSE = new Pose(8, 136, 0);
    public static final Pose ROTATED_BLUE_BASKET_POSE = BLUE_BASKET_POSE.rotate(BLUE_TEAM_HEADING, true);
    public static final Pose BLUE_AUDIENCE_POSE = new Pose(57, 9.3, Math.PI/2);
    public static final Pose BLUE_AUDIENCE_SHOOT_POSE = new Pose(58, 21, Math.toRadians(112.5));
    public static final Pose BLUE_GOAL_POSE = new Pose(21.9, 121.1,3 * Math.PI / 4);
    public static final Pose BLUE_RAMP_POSE = new Pose(16.7, 62.05, 0); // heading = -0.162
    public static double GATEPICK_LENGTH_BACKUP_X = -2.2;
    public static double GATEPICK_LENGTH_BACKUP_Y = -2.9;
    public static final Pose BLUE_RAMP_END_POSE = new Pose(BLUE_RAMP_POSE.getX() + GATEPICK_LENGTH_BACKUP_X, BLUE_RAMP_POSE.getY() + GATEPICK_LENGTH_BACKUP_Y, -0.71);
    public static final Pose BLUE_OPEN_RAMP_PICK_POSE = new Pose(14, 58.5, -0.75);
    public static final Pose CLOSE_SHOOT_BLUE_POSE = new Pose(48.3, 95, Math.toRadians(134));
    public static final Pose LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE = new Pose(55, 85, Math.toRadians(61));
    public static final double CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY = linearSpeedFromPedroRange(CLOSE_SHOOT_BLUE_POSE.distanceFrom(BLUE_BASKET_POSE));
    public static final double AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY = linearSpeedFromPedroRange(BLUE_AUDIENCE_SHOOT_POSE.distanceFrom(BLUE_BASKET_POSE));
    public static final double INIT_SHOOT_AUTO_SHOOTER_VELOCITY = 3;
    public static final Pose FIRST_ROW_BLUE_POSE = new Pose(43.2, 82, 0);
    public static final Pose SECOND_ROW_BLUE_POSE = FIRST_ROW_BLUE_POSE
            .withY(FIRST_ROW_BLUE_POSE.getY() - TILE_DIM);
    public static final Pose THIRD_ROW_BLUE_POSE = SECOND_ROW_BLUE_POSE
            .withY(SECOND_ROW_BLUE_POSE.getY() - TILE_DIM * 1.04);
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

    public static final Pose CLOSE_SHOOT_RED_POSE = CLOSE_SHOOT_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose LOOK_MOTIF_CLOSE_SHOOT_RED_POSE = LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose FIRST_ROW_RED_POSE = FIRST_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose SECOND_ROW_RED_POSE = SECOND_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose THIRD_ROW_RED_POSE = THIRD_ROW_BLUE_POSE.mirror(FIELD_SIDE_LENGTH);

    public static final Pose RED_VOID_POSE = BLUE_VOID_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final Pose RED_WALL_PICK_POSE = BLUE_WALL_PICK_POSE.mirror(FIELD_SIDE_LENGTH);
    public static final double LENGTH_WALL_PICK = 20;

    public enum Team {
        BLUE(ROTATED_RESET_POSE_BLUE,
                BLUE_GOAL_POSE,
                BLUE_AUDIENCE_POSE,
                CLOSE_SHOOT_BLUE_POSE,
                BLUE_AUDIENCE_SHOOT_POSE,
                FIRST_ROW_BLUE_POSE,
                SECOND_ROW_BLUE_POSE,
                THIRD_ROW_BLUE_POSE,
                BLUE_RAMP_POSE,
                BLUE_RAMP_END_POSE,
                BLUE_BASKET_POSE,
                BLUE_OPEN_RAMP_PICK_POSE, BLUE_VOID_POSE, LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE, BLUE_WALL_PICK_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY),
        RED(RESET_POSE_RED,
                RED_GOAL_POSE,
                RED_AUDIENCE_POSE,
                CLOSE_SHOOT_RED_POSE,
                RED_AUDIENCE_SHOOT_POSE,
                FIRST_ROW_RED_POSE,
                SECOND_ROW_RED_POSE,
                THIRD_ROW_RED_POSE,
                RED_RAMP_POSE,
                RED_RAMP_END_POSE,
                RED_BASKET_POSE,
                RED_OPEN_RAMP_PICK_POSE, RED_VOID_POSE, LOOK_MOTIF_CLOSE_SHOOT_RED_POSE, RED_WALL_PICK_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY),

        NONE(FIELD_CENTER_90HEADING_POSE, null, null, null,null , null, null, null,null, null, null, null, null, null, null, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY);

        private final Pose resetPose;
        private final Pose goalInitPose;
        private final Pose audienceInitPose;

        private final Pose closeShootPose;
        private final Pose audienceShootPose;
        private final Pose firstRowPose;
        private final Pose secondRowPose;

        private final Pose thirdRowPose;
        private final Pose rampPose;
        private final Pose endRampPose;
        private final Pose basketPose;
        private final Pose openRampPickPose;
        private final Pose voidPose;
        private final Pose lookMotifPose;
        private final Pose wallPickPose;
        private final double closeShootVelocity;
        private final double audienceShootVelocity;


        Team(Pose resetPose, Pose goalInitPose, Pose audienceInitPose, Pose closeShootPose, Pose audienceShootPose, Pose firstRowPose, Pose secondRowPose, Pose thirdRowPose, Pose rampPose, Pose endRampPose, Pose basketPose, Pose openRampPickPose, Pose voidPose, Pose lookMotifPose, Pose wallPickPose, double closeShootVelocity, double audienceShootVelocity) {
            this.resetPose = resetPose;
            this.goalInitPose = goalInitPose;
            this.audienceInitPose = audienceInitPose;
            this.closeShootPose = closeShootPose;
            this.audienceShootPose = audienceShootPose;
            this.firstRowPose = firstRowPose;
            this.secondRowPose = secondRowPose;
            this.thirdRowPose = thirdRowPose;
            this.rampPose = rampPose;
            this.endRampPose = endRampPose;
            this.basketPose = basketPose;
            this.openRampPickPose = openRampPickPose;
            this.voidPose = voidPose;
            this.lookMotifPose = lookMotifPose;
            this.wallPickPose = wallPickPose;
            this.closeShootVelocity = closeShootVelocity;
            this.audienceShootVelocity = audienceShootVelocity;
        }

        public Pose getWallPickPose() {
            return wallPickPose;
        }

        public Pose getLookMotifPose() {
            return lookMotifPose;
        }

        public double getCloseShootVelocity() {
            return closeShootVelocity;
        }

        public double getAudienceShootVelocity() {
            return audienceShootVelocity;
        }

        public Pose getCloseShootPose() {
            return closeShootPose;
        }

        public Pose getAudienceShootPose() {
            return audienceShootPose;
        }

        public Pose getFirstRowPose(){return firstRowPose;}

        public Pose getSecondRowPose(){return secondRowPose;}

        public Pose getThirdRowPose(){return thirdRowPose;}

        public Pose getRampPose(){return rampPose;}

        public Pose getEndRampPose(){return endRampPose;}


        public Pose getResetPose() {return resetPose;}
        public Pose getGoalInitPose() {return goalInitPose;}
        public Pose getAudienceInitPose() {return audienceInitPose;}
        public Pose getBasketPose() {return basketPose;}
        public Pose getOpenRampPickPose() {return openRampPickPose;}
        public Pose getVoidPose(){return voidPose;}
    }


}
