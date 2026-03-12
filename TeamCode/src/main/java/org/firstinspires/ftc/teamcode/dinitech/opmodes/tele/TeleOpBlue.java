package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FlipFieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends TeleOpBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);

            if (PoseStorage.getLastPose() == null){
                drivePedroSubsystem.getDrive().setPose(hubsSubsystem.getTeam().getResetPose());
            } else {
                drivePedroSubsystem.setPose(PoseStorage.getLastPose().rotate(BLUE_TEAM_HEADING, true));
                PoseStorage.clearLastPose();
            }
    }
}
