package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

public class BlueGoalAutoBase extends AutoBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
            drivePedroSubsystem.getDrive().prepAuto(hubsSubsystem.getTeam().getGoalInitPose());
    }
}
