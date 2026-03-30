package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

@Autonomous(name = "RedGoalThreeRows", group = "Red")
public class RedGoalThreeRows extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();
        new ThreeRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, 1).schedule();
    }
}
