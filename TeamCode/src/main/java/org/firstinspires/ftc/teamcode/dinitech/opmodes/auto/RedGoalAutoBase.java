package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.bases;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_GOAL_POSE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class RedGoalAutoBase extends AutoBase {

    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);
            drivePedroSubsystem.getDrive().prepAuto(RED_GOAL_POSE);

    }

    @Override
    public void run() {
            super.run();
    }

}
