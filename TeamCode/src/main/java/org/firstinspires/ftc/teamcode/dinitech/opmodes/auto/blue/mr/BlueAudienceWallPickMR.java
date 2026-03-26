package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.WallPickFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;

@Autonomous(name = "BlueAudienceWallPickMR", group = "Blue")
//@Disabled

public class BlueAudienceWallPickMR extends BlueAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new WallPickFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS/2).schedule();
    }
}
