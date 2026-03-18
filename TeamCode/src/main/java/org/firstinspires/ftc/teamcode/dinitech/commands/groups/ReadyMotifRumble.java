package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ReadyMotifRumble extends ReadyMotif {

    private final GamepadSubsystem gamepadSubsystem;

    public ReadyMotifRumble(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        super(trieurSubsystem, visionSubsystem);
        this.gamepadSubsystem = gamepadSubsystem;
    }

    @Override
    protected void onCantMotif() {
        gamepadSubsystem.customRumble(gamepadSubsystem.cantMotifRumble, 3, true);
    }
}