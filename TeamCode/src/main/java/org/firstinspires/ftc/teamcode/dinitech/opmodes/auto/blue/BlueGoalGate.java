package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoGatePickFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueGoalGate", group = "Blue")
public class BlueGoalGate extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new TwoGatePickFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, hubsSubsystem).schedule();
    }



}
