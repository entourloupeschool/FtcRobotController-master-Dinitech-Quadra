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
    List<Blinker.Step> patternBlueDecay = new ArrayList<>();


    List<Blinker.Step> patternRed = new ArrayList<>();
    List<Blinker.Step> patternRedOpen = new ArrayList<>();
    List<Blinker.Step> patternRedDecay = new ArrayList<>();


    // Helper methods to create color int from RGB/ARGB values
    private static int rgb(int r, int g, int b) {
        return (r << 16) | (g << 8) | b;
    }

    private static int argb(int a, int r, int g, int b) {
        return (a << 24) | (r << 16) | (g << 8) | b;
    }

    private int green = rgb(0, 255, 0);
    private int blue = argb(255, 0, 0, 255);

    private int red = argb(255, 255, 0, 0);
    private int yellow = rgb(255, 255, 0);
    private int orange = rgb(255, 165, 0);


    public DefaultHubsCommand(HubsSubsystem hubsSubsystem, TrieurSubsystem trieurSubsystem, BooleanSupplier isBlueSupplier) {
        this.hubsSubsystem = hubsSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.isBlueSupplier = isBlueSupplier;
        addRequirements(hubsSubsystem);
    }

    @Override
    public void initialize() {
        patternBlue.add(new Blinker.Step(blue, 5000, TimeUnit.MILLISECONDS));

        patternBlueOpen.add(new Blinker.Step(blue, 20, TimeUnit.MILLISECONDS));
        patternBlueOpen.add(new Blinker.Step(yellow, 20, TimeUnit.MILLISECONDS));
        
        for (int i = 0; i < 255; i += 15) {
            patternBlueDecay.add(new Blinker.Step(rgb(255 - i, 255 - i, i), 8, TimeUnit.MILLISECONDS));
            patternRedDecay.add(new Blinker.Step(rgb(255, 255 - i, 255 - i), 8, TimeUnit.MILLISECONDS));
        }
        for (int i = 255; i > 0; i -= 15) {
            patternBlueDecay.add(new Blinker.Step(rgb(255 - i, 255 - i, i), 12, TimeUnit.MILLISECONDS));
            patternRedDecay.add(new Blinker.Step(rgb(255, 255 - i, 255 - i), 12, TimeUnit.MILLISECONDS));
        }

        patternRed.add(new Blinker.Step(red, 5000, TimeUnit.MILLISECONDS));

        patternRedOpen.add(new Blinker.Step(red, 20, TimeUnit.MILLISECONDS));
        patternRedOpen.add(new Blinker.Step(yellow, 20, TimeUnit.MILLISECONDS));

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
                hubsSubsystem.setPattern(patternBlueDecay);
            }
        } else {
            if (trappeOpen) {
                hubsSubsystem.setPattern(patternRedOpen);
            } else {
                hubsSubsystem.setPattern(patternRedDecay);
            }
        }
    }
}
