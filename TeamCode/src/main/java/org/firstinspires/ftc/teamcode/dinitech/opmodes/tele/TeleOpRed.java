package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

@TeleOp(name = "TeleOpRed", group = "TeleOp")
public class TeleOpRed extends TeleOpBase {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);
    }

    @Override
    public void run() {
            super.run();
    }
}
