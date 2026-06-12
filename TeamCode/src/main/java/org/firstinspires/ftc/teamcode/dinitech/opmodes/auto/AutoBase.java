package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.FOLLOWER_T_POSITION_END;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.OpenFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OnlyMotifDetection;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.Gornetix;
import org.firstinspires.ftc.teamcode.dinitech.other.MoulinPositionColorsStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class AutoBase extends Gornetix {
    public VisionSubsystem visionSubsystem;

    private int lastHowManyArtefacts = 0;

    @Override
    public void initialize() {
            super.initialize();

            if (shouldInitializeVisionSubsystem()) {
                visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
                register(visionSubsystem);
                visionSubsystem.setDefaultCommand(new OnlyMotifDetection(visionSubsystem));
            }

            drivePedroSubsystem.setFollowerTEnd(FOLLOWER_T_POSITION_END);

            drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);

            autoSetArtefactColors();

            MoulinPositionColorsStorage.setLastMoulinPositionColors(trieurSubsystem.getMoulinStoragePositionColors(), trieurSubsystem.getHowManyArtefacts());
            lastHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();

            trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT);

            schedule(new SequentialCommandGroup(
                    new InstantCommand(),
                    new OpenFinger(trieurSubsystem)));
    }

    @Override
    public void run() {
            // save pose to pose storage
            PoseStorage.setLastPose(drivePedroSubsystem.getPose());

            int currentHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();
            if(currentHowManyArtefacts != lastHowManyArtefacts){
                lastHowManyArtefacts = currentHowManyArtefacts;
                MoulinPositionColorsStorage.setLastMoulinPositionColors(trieurSubsystem.getMoulinStoragePositionColors(), trieurSubsystem.getHowManyArtefacts());
            }

            if (visionSubsystem != null && MotifStorage.getMotifNumber() == -1){
                if (visionSubsystem.hasMotif()) MotifStorage.setMotifNumber(visionSubsystem.getCachedMotif());
            }

            super.run();
    }

    private void autoSetArtefactColors(){
            trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.GREEN);
            trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setMoulinStoragePositionColor(5, TrieurSubsystem.ArtifactColor.PURPLE);
            trieurSubsystem.setHowManyArtefacts(3);
    }

    protected boolean shouldInitializeVisionSubsystem() {
            return true;
    }
}
