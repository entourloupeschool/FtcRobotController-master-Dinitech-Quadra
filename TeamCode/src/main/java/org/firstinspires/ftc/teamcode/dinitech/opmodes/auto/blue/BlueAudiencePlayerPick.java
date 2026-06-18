package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;



import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;

@Autonomous(name = "BlueAudienceThreeRows", group = "Blue")
public class BlueAudienceThreeRows extends BlueAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();
    }
}
