package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;



import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.PlayerPickFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueAudienceAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedAudienceAutoBase;

@Autonomous(name = "RedAudiencePlayerPick", group = "Red")
public class RedAudiencePlayerPick extends RedAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();
            new PlayerPickFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();
    }
}
