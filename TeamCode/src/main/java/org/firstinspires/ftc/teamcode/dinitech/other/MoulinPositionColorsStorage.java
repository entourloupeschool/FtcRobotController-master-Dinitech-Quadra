package org.firstinspires.ftc.teamcode.dinitech.other;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public final class MoulinPositionColorsStorage {
    private static TrieurSubsystem.ArtifactColor[] moulinStoragePositionColors = null;

    public static void setLastMoulinPositionColors(TrieurSubsystem.ArtifactColor[] newMPC) {
        moulinStoragePositionColors = newMPC;
    }

    public static TrieurSubsystem.ArtifactColor[] getLastMoulinPositionColors() {
        return moulinStoragePositionColors;
    }

    public static void clearLastMoulinPositionColors() {
        moulinStoragePositionColors = null;
    }
}
