package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.BlueThreeRowsFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;

@Autonomous(name = "BlueAudienceThreeRowsSlow", group = "Blue")
public class BlueAudienceThreeRowsSlow extends BlueAudienceAutoBase {


    @Override
    public void initialize() {
            super.initialize();

        trieurSubsystem.setWantsMotifShoot(true);

        new BlueThreeRowsFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, gamepadSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();

    }

    @Override
    public void run() {
            super.run();
    }
}
