package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;



import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.ThreeRowsFromGoalWithGateOpen;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "BlueGoalThreeRowsWithGate", group = "Blue")
public class BlueGoalThreeRowsGate extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

        new ThreeRowsFromGoalWithGateOpen(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();
    }
}
