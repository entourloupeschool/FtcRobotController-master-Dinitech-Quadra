package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromAudience;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedAudienceAutoBase;

@Autonomous(name = "RedAudienceThreeRows", group = "Red")
public class RedAudienceThreeRows extends RedAudienceAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromAudience(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, AUTO_ROBOT_CONSTRAINTS).schedule();
    }
}
