package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collection;
import java.util.List;

public class HubsSubsystem extends SubsystemBase {
    private final List<LynxModule> hubs;

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
