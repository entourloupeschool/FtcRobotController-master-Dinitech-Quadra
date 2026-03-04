package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.dinitech.commands.SetDefault;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OnlyMotifDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.Gornetix;
import org.firstinspires.ftc.teamcode.dinitech.other.MoulinPositionColorsStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class AutoBase extends Gornetix {
    private int lastHowManyArtefacts = 0;
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
            visionSubsystem.setDefaultCommand(new OnlyMotifDetections(visionSubsystem));

            autoSetArtefactColors();
            trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT);

            new Trigger(trieurSubsystem::getIsFull)
                    .whenActive(new StopChargeur(chargeurSubsystem));

            new Trigger(()-> trieurSubsystem.getHowManyArtefacts() != lastHowManyArtefacts).whenActive(() -> {
                        lastHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();
                        MoulinPositionColorsStorage.setLastMoulinPositionColors(trieurSubsystem.getMoulinStoragePositionColors());});

            new Trigger(() -> visionSubsystem.hasColorOrder()).whenActive(()->
                    MotifStorage.setMotifNumber(visionSubsystem.getCachedMotif()));
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            // save pose to pose storage
            PoseStorage.setLastPose(drivePedroSubsystem.getPose());

            super.run();
    }

    /**
     * auto set artefact colors
     */
    private void autoSetArtefactColors(){
            trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.GREEN);
            trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setMoulinStoragePositionColor(5, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setHowManyArtefacts(3);
    }
}
