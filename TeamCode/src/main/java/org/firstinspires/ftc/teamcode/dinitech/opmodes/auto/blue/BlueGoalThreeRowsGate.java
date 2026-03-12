package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueGoalThreeRows", group = "Blue")
public class BlueGoalThreeRows extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, gamepadSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }
}
