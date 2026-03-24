package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RESET_POSE_BLUE;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

public class BlueTestResetPoseAutoBase extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
            drivePedroSubsystem.getDrive().prepAuto(RESET_POSE_BLUE);
    }
}
