package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class BlueGoalAuto extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);
            drivePedroSubsystem.getDrive().prepAuto(BLUE_GOAL_POSE);

    }

    @Override
    public void run() {
            super.run();
    }
}
