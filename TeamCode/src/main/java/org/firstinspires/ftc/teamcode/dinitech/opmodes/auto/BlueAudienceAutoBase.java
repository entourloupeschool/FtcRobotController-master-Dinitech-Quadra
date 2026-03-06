package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.bases;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_POSE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class BlueAudienceAutoBase extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);
            drivePedroSubsystem.getDrive().prepAuto(BLUE_AUDIENCE_POSE);

    }

    @Override
    public void run() {
            super.run();
    }
}
