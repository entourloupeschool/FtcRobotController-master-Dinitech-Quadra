package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue.mr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.TwoGateFromGoalMR;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueGoalGateTwoRowsMR", group = "Blue")
public class BlueGoalGateTwoRowsMR extends BlueGoalAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

        new TwoGateFromGoalMR(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
