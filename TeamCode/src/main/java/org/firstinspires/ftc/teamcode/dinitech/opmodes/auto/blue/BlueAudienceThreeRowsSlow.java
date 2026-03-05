package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ThreeRowsFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.bases.BlueAudienceAutoBase;

@Autonomous(name = "BlueAudienceThreeRowsSlow", group = "Blue")
public class BlueAudienceThreeRowsSlow extends BlueAudienceAutoBase {


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

        trieurSubsystem.setWantsMotifShoot(true);

        new ThreeRowsFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, gamepadSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
