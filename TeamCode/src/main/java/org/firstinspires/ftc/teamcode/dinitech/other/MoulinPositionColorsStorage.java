package org.firstinspires.ftc.teamcode.dinitech.other;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public final class MoulinPositionColorsStorage {
    private static TrieurSubsystem.ArtifactColor[] moulinStoragePositionColors = null;
    private static int howManyArtefactStorage = 0;

    public static void setLastMoulinPositionColors(TrieurSubsystem.ArtifactColor[] newMPC, int howManyArtefacts) {
        moulinStoragePositionColors = newMPC;
        howManyArtefactStorage = howManyArtefacts;
    }

    public static TrieurSubsystem.ArtifactColor[] getLastMoulinPositionColors() {
        return moulinStoragePositionColors;
    }

    public static int getHowManyArtefactStorage(){
        return howManyArtefactStorage;
    }

    public static void clearHowManyArtefactStorage(){
        howManyArtefactStorage = 0;
    }

    public static void clearLastMoulinPositionColors() {
        moulinStoragePositionColors = null;
    }
}
