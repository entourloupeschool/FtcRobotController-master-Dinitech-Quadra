package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.TwoRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

//@Autonomous(name = "BlueGoalTwoRowsMR", group = "Blue")
@Disabled

public class BlueGoalTwoRowsMR extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new TwoRowsFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }
}
