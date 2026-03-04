package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class BlueAuto extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);
    }

    @Override
    public void run() {
            super.run();
    }
}
