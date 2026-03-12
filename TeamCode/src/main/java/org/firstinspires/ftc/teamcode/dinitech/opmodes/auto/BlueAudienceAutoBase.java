package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_POSE;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class BlueAudienceAutoBase extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
            drivePedroSubsystem.getDrive().prepAuto(hubsSubsystem.getTeam().getAudienceInitPose());

    }
}
