package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

@TeleOp(name = "TeleOpRed", group = "TeleOp")
public class TeleOpRed extends TeleOpBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.RED);
    }

    @Override
    public void run() {
            super.run();
    }
}
