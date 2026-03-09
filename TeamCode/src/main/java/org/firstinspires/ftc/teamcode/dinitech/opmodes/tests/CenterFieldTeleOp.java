package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.tele.TeleOpBase;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

@TeleOp(name = "CenterFieldTeleOp", group = "Tests")
public class CenterFieldTeleOp extends TeleOpBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
            drivePedroSubsystem.getDrive().setPose(FIELD_CENTER_90HEADING_POSE);
    }

    @Override
    public void run() {
            super.run();
    }
}
