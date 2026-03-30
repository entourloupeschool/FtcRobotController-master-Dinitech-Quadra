package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromGoalWithGateOpen;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

@Autonomous(name = "RedGoalThreeRowsWithGate", group = "Red")
public class RedGoalThreeRowsGate extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromGoalWithGateOpen(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 0.5).schedule();
    }
}
