package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

import java.util.Collection;
import java.util.List;

@Configurable
public class HubsSubsystem extends SubsystemBase {
    public static long TELEMETRY_UPDATE_INTERVAL_MS = 500;
    private final List<LynxModule> hubs;


    private TeamPoses.Team team = TeamPoses.Team.NONE;

    public TeamPoses.Team getTeam(){
        return team;
    }

    public void setTeam(TeamPoses.Team team){
        this.team = team;
    }


    public HubsSubsystem(HardwareMap hardwareMap){
        hubs = hardwareMap.getAll(LynxModule.class);
        init();
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
