package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_TEAM_HEADING;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FlipFieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

@TeleOp(name = "GornetixTeleOpRed - Dinitech", group = "TeleOp")
public class GornetixTeleOpRed extends GornetixTeleOp {
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
