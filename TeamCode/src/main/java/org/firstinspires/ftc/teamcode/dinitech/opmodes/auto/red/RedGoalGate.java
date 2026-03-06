package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.BlueTwoGateFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;

@Autonomous(name = "RedGoalGate", group = "Red")
public class RedGoalGate extends RedGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new BlueTwoGateFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem).schedule();
    }


    @Override
    public void run() {
            super.run();
    }


}
