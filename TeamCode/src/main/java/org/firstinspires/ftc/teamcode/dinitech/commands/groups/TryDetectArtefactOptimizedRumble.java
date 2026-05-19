package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class TryDetectArtefactRumble extends TryDetectArtefact {

    private final GamepadSubsystem gamepadSubsystem;

    public TryDetectArtefactRumble(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        super(trieurSubsystem);
        this.gamepadSubsystem = gamepadSubsystem;
    }

    @Override
    protected void onTimeoutReached() {
        gamepadSubsystem.customRumble(gamepadSubsystem.unfoundRumbleEffect, 2, true);
    }

    @Override
    protected void onWaitingForArtefact() {
        gamepadSubsystem.customRumble(gamepadSubsystem.waitRumbleEffect, 2, true);
    }
}