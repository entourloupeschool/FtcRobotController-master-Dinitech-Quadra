package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class DefaultHubsCommand extends CommandBase {
    private final HubsSubsystem hubsSubsystem;
    private final TrieurSubsystem trieurSubsystem;
    private final BooleanSupplier isBlueSupplier;

    private boolean lastTrappeOpen = false;
    private boolean lastIsBlue = false;
    private boolean patternSet = false;

    List<Blinker.Step> patternBlue = new ArrayList<>();
    List<Blinker.Step> patternBlueOpen = new ArrayList<>();

    List<Blinker.Step> patternRed = new ArrayList<>();
    List<Blinker.Step> patternRedOpen = new ArrayList<>();



    public DefaultHubsCommand(HubsSubsystem hubsSubsystem, TrieurSubsystem trieurSubsystem, BooleanSupplier isBlueSupplier) {
        this.hubsSubsystem = hubsSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.isBlueSupplier = isBlueSupplier;
        addRequirements(hubsSubsystem);
    }

    @Override
    public void initialize() {
        patternBlue.add(new Blinker.Step(0xFF0000FF, 5000, TimeUnit.MILLISECONDS));  // Blue for 500ms

        patternBlueOpen.add(new Blinker.Step(0xFF00FF00, 50, TimeUnit.MILLISECONDS));  // Green for 500ms
        patternBlueOpen.add(new Blinker.Step(0, 50, TimeUnit.MILLISECONDS));  // Green for 500ms

        patternRed.add(new Blinker.Step(0xFFFF0000, 5000, TimeUnit.MILLISECONDS));  // Red for 500ms

        patternRedOpen.add(new Blinker.Step(0xFFFF0000, 50, TimeUnit.MILLISECONDS));  // Red for 500ms
        patternRedOpen.add(new Blinker.Step(0, 50, TimeUnit.MILLISECONDS));  // Red for 500ms

        // Set initial pattern
        updatePattern();
    }
    @Override
    public void execute() {
        boolean currentTrappeOpen = trieurSubsystem.isTrappeOpen();
        boolean currentIsBlue = isBlueSupplier.getAsBoolean();

        // Only update pattern when trappe state or team color changes
        if (!patternSet || currentTrappeOpen != lastTrappeOpen || currentIsBlue != lastIsBlue) {
            updatePattern();
            lastTrappeOpen = currentTrappeOpen;
            lastIsBlue = currentIsBlue;
            patternSet = true;
        }
    }

    private void updatePattern() {
        boolean trappeOpen = trieurSubsystem.isTrappeOpen();
        boolean isBlue = isBlueSupplier.getAsBoolean();

        if (isBlue) {
            if (trappeOpen) {
                hubsSubsystem.setPattern(patternBlueOpen);
            } else {
                hubsSubsystem.setPattern(patternBlue);
            }
        } else {
            if (trappeOpen) {
                hubsSubsystem.setPattern(patternRedOpen);
            } else {
                hubsSubsystem.setPattern(patternRed);
            }
        }
    }

}
