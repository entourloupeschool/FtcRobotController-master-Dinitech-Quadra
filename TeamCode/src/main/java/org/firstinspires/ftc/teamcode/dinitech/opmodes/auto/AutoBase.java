package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OnlyMotifDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.Gornetix;
import org.firstinspires.ftc.teamcode.dinitech.other.MoulinPositionColorsStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class AutoBase extends Gornetix {
    private int lastHowManyArtefacts = 0;

    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
            visionSubsystem.setDefaultCommand(new OnlyMotifDetections(visionSubsystem));

            autoSetArtefactColors();
            MoulinPositionColorsStorage.setLastMoulinPositionColors(trieurSubsystem.getMoulinStoragePositionColors());
            lastHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();

            trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT);
    }

    @Override
    public void run() {
            // save pose to pose storage
            PoseStorage.setLastPose(drivePedroSubsystem.getPose());

            int currentHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();
            if(currentHowManyArtefacts != lastHowManyArtefacts){
                lastHowManyArtefacts = currentHowManyArtefacts;
                MoulinPositionColorsStorage.setLastMoulinPositionColors(trieurSubsystem.getMoulinStoragePositionColors());
                if (currentHowManyArtefacts == 3) {
                    chargeurSubsystem.setChargeurPower(0);
                }
            }

            if (MotifStorage.getMotifNumber() == -1){
                if (visionSubsystem.hasColorOrder()) MotifStorage.setMotifNumber(visionSubsystem.getCachedMotif());
            }

            super.run();
    }

    private void autoSetArtefactColors(){
            trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.GREEN);
            trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setMoulinStoragePositionColor(5, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setHowManyArtefacts(3);
    }
}
