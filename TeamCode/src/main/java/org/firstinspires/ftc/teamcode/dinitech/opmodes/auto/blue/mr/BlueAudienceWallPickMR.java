package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.WallPickFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;

@Autonomous(name = "BlueAudienceWallPickMR", group = "Blue")
//@Disabled

public class BlueAudienceWallPickMR extends BlueAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new WallPickFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 0.5).schedule();
    }
}
