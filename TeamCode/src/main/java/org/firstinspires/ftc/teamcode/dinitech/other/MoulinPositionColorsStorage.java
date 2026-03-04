package org.firstinspires.ftc.teamcode.dinitech.other;

import com.pedropathing.geometry.Pose;

public final class PoseStorage {
    public static Pose lastPose = null;

    public static void setLastPose(Pose pose) {
        lastPose = pose;
    }

    public static Pose getLastPose() {
        return lastPose;
    }

    public static void clearLastPose() {
        lastPose = null;
    }
}
