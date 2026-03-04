package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class BlueAuto extends AutoBase {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }
}
