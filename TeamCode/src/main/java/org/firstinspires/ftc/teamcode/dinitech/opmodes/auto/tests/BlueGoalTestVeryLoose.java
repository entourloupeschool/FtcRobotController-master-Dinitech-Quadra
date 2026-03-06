package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToMotifShoot;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

//@Autonomous(name = "TestVeryLoose - Dinitech", group = "Test")
@Disabled

public class BlueGoalTestVeryLoose extends BlueGoalAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            new InitToMotifShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, gamepadSubsystem, CLOSE_SHOOT_BLUE_POSE).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
