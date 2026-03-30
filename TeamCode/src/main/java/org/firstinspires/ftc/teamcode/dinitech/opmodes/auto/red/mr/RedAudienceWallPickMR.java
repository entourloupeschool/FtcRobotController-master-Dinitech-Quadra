package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red.mr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.WallPickFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedAudienceAutoBase;

@Autonomous(name = "RedAudienceWallPickMR", group = "Red")
//@Disabled

public class RedAudienceWallPickMR extends RedAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new WallPickFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 0.5).schedule();
    }
}
