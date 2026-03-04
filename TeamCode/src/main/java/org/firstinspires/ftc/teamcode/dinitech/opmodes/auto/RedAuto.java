package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;


import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class RedAuto extends AutoBase {

    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);
    }

    @Override
    public void run() {
            super.run();
    }

}
