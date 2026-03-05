package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ThreeRowsFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.bases.RedGoalAutoBase;

@Autonomous(name = "RedGoalThreeRows", group = "Red")
public class RedGoalThreeRows extends RedGoalAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();
        new ThreeRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, gamepadSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
