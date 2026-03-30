package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red.mr;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.ThreeRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

//@Autonomous(name = "RedGoalGateThreeRowsMR", group = "Red")
@Disabled

public class RedGoalGateThreeRowsMR extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 1).schedule();
    }
}
