package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUDIENCE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_OPEN_RAMP_PICK_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_VOID_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LOOK_MOTIF_CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_OPEN_RAMP_PICK_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_VOID_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_RED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROTATED_RESET_POSE_BLUE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_RED_POSE;

import com.pedropathing.geometry.Pose;

public class TeamPoses {
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
                BLUE_OPEN_RAMP_PICK_POSE, BLUE_VOID_POSE, LOOK_MOTIF_CLOSE_SHOOT_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_AUTO_SHOOTER_VELOCITY),
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
                RED_OPEN_RAMP_PICK_POSE, RED_VOID_POSE, LOOK_MOTIF_CLOSE_SHOOT_RED_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_AUTO_SHOOTER_VELOCITY),

        NONE(FIELD_CENTER_90HEADING_POSE, null, null, null,null , null, null, null,null, null, null, null, null, null, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, AUDIENCE_AUTO_SHOOTER_VELOCITY);

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
        private final double closeShootVelocity;
        private final double audienceShootVelocity;


        Team(Pose resetPose, Pose goalInitPose, Pose audienceInitPose, Pose closeShootPose, Pose audienceShootPose, Pose firstRowPose, Pose secondRowPose, Pose thirdRowPose, Pose rampPose, Pose endRampPose, Pose basketPose, Pose openRampPickPose, Pose voidPose, Pose lookMotifPose, double closeShootVelocity, double audienceShootVelocity) {
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
            this.closeShootVelocity = closeShootVelocity;
            this.audienceShootVelocity = audienceShootVelocity;
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
