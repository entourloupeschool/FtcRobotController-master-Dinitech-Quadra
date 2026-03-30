package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red.mr;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.TwoRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

//@Autonomous(name = "RedGoalTwoRowsMR", group = "Red")
@Disabled
public class RedGoalTwoRowsMR extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new TwoRowsFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 1).schedule();
    }
}
