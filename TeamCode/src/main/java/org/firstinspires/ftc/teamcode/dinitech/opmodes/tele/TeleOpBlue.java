package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.BLUE_TEAM_HEADING;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FlipFieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends TeleOpBaseAutomations {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);

            if (PoseStorage.getLastPose() == null){
                drivePedroSubsystem.setPose(hubsSubsystem.getTeam().getResetPose());

            } else {
                drivePedroSubsystem.setPose(PoseStorage.getLastPose().rotate(BLUE_TEAM_HEADING, true));
                PoseStorage.clearLastPose();

            }
    }
}
