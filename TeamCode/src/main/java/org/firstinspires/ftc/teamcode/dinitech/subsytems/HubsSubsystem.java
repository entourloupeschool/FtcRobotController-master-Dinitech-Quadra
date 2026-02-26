package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_GOAL_POSE;

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
        BLUE,
        RED,
        NONE
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
                return RED_GOAL_POSE;
            default:
                return FIELD_CENTER_90HEADING_POSE;
        }
    }

    public HubsSubsystem(HardwareMap hardwareMap, boolean isBlue){
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
