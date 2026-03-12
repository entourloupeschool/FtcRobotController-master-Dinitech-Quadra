package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.ThreeRowsFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

//@Autonomous(name = "BlueGoalGateThreeRowsMR", group = "Blue")
@Disabled
public class BlueGoalGateThreeRowsMR extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, gamepadSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }
}
