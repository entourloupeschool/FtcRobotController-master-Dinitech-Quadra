package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.TwoRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

//@Autonomous(name = "RedGoalTwoRowsMR", group = "Red")
@Disabled
public class RedGoalTwoRowsMR extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new TwoRowsFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }
}
