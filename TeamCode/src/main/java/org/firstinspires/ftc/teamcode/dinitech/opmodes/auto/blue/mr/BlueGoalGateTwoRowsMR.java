package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr.TwoGateFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

//@Autonomous(name = "BlueGoalGateTwoRowsMR", group = "Blue")
@Disabled

public class BlueGoalGateTwoRowsMR extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new TwoGateFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem).schedule();
    }

}
