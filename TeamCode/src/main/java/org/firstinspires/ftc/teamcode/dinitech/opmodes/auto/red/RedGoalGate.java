package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoGateFromGoalGilles;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoGateFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

@Autonomous(name = "RedGoalGate", group = "Red")
public class RedGoalGate extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new TwoGateFromGoalGilles(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem).schedule();
    }
}
