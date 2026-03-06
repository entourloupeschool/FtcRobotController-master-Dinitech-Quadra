package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_AUDIENCE_POSE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class RedAudienceAutoBase extends AutoBase {

    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);
            drivePedroSubsystem.getDrive().prepAuto(RED_AUDIENCE_POSE);

    }

    @Override
    public void run() {
            super.run();
    }

}
