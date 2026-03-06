package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_RED_POSE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collection;
import java.util.List;

public class HubsSubsystem extends SubsystemBase {
    private final List<LynxModule> hubs;
    private final Pose ROTATED_BLUE_BASKET_POSE = BLUE_BASKET_POSE.rotate(BLUE_TEAM_HEADING, false);


    public enum Team {
        BLUE(FIELD_CENTER_90HEADING_POSE,
                BLUE_GOAL_POSE,
                BLUE_AUDIENCE_POSE,
                CLOSE_SHOOT_BLUE_POSE,
                BLUE_AUDIENCE_SHOOT_POSE,
                FIRST_ROW_BLUE_POSE,
                SECOND_ROW_BLUE_POSE,
                THIRD_ROW_BLUE_POSE,
                BLUE_RAMP_POSE,
                BLUE_RAMP_END_POSE),
        RED(FIELD_CENTER_90HEADING_POSE,
                RED_GOAL_POSE,
                RED_AUDIENCE_POSE,
                CLOSE_SHOOT_RED_POSE,
                RED_AUDIENCE_SHOOT_POSE,
                FIRST_ROW_RED_POSE,
                SECOND_ROW_RED_POSE,
                THIRD_ROW_RED_POSE,
                RED_RAMP_POSE,
                RED_RAMP_END_POSE),

        NONE(FIELD_CENTER_90HEADING_POSE, null, null, null,null , null, null, null,null, null);

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
        
        Team(Pose resetPose, Pose goalInitPose, Pose audienceInitPose, Pose closeShootPose, Pose audienceShootPose, Pose firstRowPose, Pose secondRowPose, Pose thirdRowPose, Pose rampPose, Pose endRampPose) {
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

    }

    private Team team = Team.NONE;

    public Team getTeam(){
        return team;
    }

    public void setTeam(Team team){
        this.team = team;
    }

    public Pose getGoalPose(){
        switch (getTeam()){
            case BLUE:
                return ROTATED_BLUE_BASKET_POSE;
            case RED:
                return RED_BASKET_POSE;
            default:
                return FIELD_CENTER_90HEADING_POSE;
        }
    }

    public HubsSubsystem(HardwareMap hardwareMap){
        hubs = hardwareMap.getAll(LynxModule.class);
        init();

        setTeam(Team.NONE);
    }

    public void init(){
        for (LynxModule hub : hubs) {
            hub.abandonUnfinishedCommands();
            hub.clearBulkCache();
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clearBulkCache(){
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void setPattern(Collection<Blinker.Step> collBlinkerStep){
        for (LynxModule hub : hubs) {
            hub.setPattern(collBlinkerStep);
        }
    }
}
