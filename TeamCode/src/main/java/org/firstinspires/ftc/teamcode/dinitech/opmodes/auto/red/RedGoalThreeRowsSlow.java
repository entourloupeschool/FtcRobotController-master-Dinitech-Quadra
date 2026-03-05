package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ThreeRowsFromGoal;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.bases.RedGoalAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

@Autonomous(name = "RedGoalThreeRowsSlow", group = "Red")
public class RedGoalThreeRowsSlow extends RedGoalAutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            trieurSubsystem.setWantsMotifShoot(true);

            new ThreeRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, hubsSubsystem, gamepadSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }

    /**
     * auto set artefact colors
     */
    private void autoSetArtefactColors(){
        trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.GREEN);
        trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.PURPLE);
        trieurSubsystem.setMoulinStoragePositionColor(5, TrieurSubsystem.ArtifactColor.PURPLE);
    }
}
