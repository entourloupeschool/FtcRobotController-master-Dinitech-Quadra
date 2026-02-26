package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_BASKET_POSE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collection;
import java.util.List;

public class HubsSubsystem extends SubsystemBase {
    private final List<LynxModule> hubs;

    private boolean onBlueTeam;
    public void setOnBlueTeam(boolean onBlueTeam){
        this.onBlueTeam = onBlueTeam;
    }

    public boolean getOnBlueTeam(){
        return onBlueTeam;
    }

    public Pose getGoalPose(){
        return getOnBlueTeam() ? BLUE_BASKET_POSE : RED_BASKET_POSE;
    }

    public HubsSubsystem(HardwareMap hardwareMap, boolean isBlue){
        hubs = hardwareMap.getAll(LynxModule.class);
        init();

        setOnBlueTeam(isBlue);
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
