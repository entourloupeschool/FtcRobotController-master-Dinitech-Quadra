package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.red;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.BlueThreeRowsFromGoal;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.RedThreeRowsFromGoal;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.RedGoalAutoBase;
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

            new RedThreeRowsFromGoal(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, visionSubsystem, chargeurSubsystem, gamepadSubsystem, MAX_POWER_ROW_PICK_ARTEFACTS).schedule();
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
