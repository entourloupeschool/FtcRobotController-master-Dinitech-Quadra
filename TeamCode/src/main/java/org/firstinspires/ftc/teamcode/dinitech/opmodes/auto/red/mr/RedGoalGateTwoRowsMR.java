package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red.mr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoGateFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

//@Autonomous(name = "RedGoalGateTwoRowsMR", group = "Red")
@Disabled

public class RedGoalGateTwoRowsMR extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new TwoGateFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem).schedule();
    }
}
