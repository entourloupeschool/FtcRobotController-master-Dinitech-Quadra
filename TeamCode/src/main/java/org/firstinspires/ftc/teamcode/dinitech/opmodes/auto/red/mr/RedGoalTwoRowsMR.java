package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoRowsFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueGoalTwoRowsMR", group = "Blue")
public class BlueGoalTwoRowsMR extends BlueGoalAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

        new TwoRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, gamepadSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
