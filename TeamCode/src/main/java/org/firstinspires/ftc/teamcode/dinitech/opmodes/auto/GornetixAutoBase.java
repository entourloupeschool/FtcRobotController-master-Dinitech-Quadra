package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixFullSystem;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class GornetixAutoBase extends GornetixFullSystem {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
            visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

            autoSetArtefactColors();

            new Trigger(trieurSubsystem::getIsFull)
                    .whenActive(new StopChargeur(chargeurSubsystem));
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            // save pose to pose storage
            PoseStorage.setLastPose(drivePedroSubsystem.getPose());
            MotifStorage.setMotifNumber(visionSubsystem.getColorsOrder());
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
