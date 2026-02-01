package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collection;
import java.util.List;

public class HubsSubsystem extends SubsystemBase {
    private List<LynxModule> hubs;


    public HubsSubsystem(HardwareMap hardwareMap){
        hubs = hardwareMap.getAll(LynxModule.class);
    }

    public void setLedColor(int color){
        for (LynxModule hub : hubs) {
            hub.setConstant(color);
        }
    }

    public void setPattern(Collection<Blinker.Step> collBlinkerStep){
        for (LynxModule hub : hubs) {
            hub.setPattern(collBlinkerStep);
        }
    }
}
