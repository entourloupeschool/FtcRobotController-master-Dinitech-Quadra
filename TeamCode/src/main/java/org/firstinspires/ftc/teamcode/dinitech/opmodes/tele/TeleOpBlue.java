package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FlipFieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

@TeleOp(name = "GornetixTeleOpBlue - Dinitech", group = "TeleOp")
public class GornetixTeleOpBlue extends GornetixTeleOp {
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);

            new FlipFieldCentricDrive(drivePedroSubsystem, BLUE_TEAM_HEADING).initialize();
    }

    @Override
    public void run() {
            super.run();
    }
}
