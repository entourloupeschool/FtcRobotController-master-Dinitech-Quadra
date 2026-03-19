package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.TwoRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.WallPickFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueAudienceWallPick", group = "Blue")
//@Disabled

public class BlueAudienceWallPick extends BlueAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new WallPickFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS/2).schedule();
    }
}
