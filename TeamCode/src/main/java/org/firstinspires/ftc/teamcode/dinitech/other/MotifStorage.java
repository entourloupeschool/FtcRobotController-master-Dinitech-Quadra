package org.firstinspires.ftc.teamcode.dinitech.other;

public final class MotifStorage {
    private static int motifNumber = -1;

    public static void setMotifNumber(int mN) {
        motifNumber = mN;
    }

    public static int getMotifNumber() {
        return motifNumber;
    }

    public static void clearMotifNumber() {
        motifNumber = -1;
    }
}
