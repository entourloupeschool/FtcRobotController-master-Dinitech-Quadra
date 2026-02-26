package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.hub;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Configurable
public class DefaultHubsCommand extends CommandBase {
    private final HubsSubsystem hubsSubsystem;
    private final TrieurSubsystem trieurSubsystem;

    private boolean lastTrappeOpen = false;
    private boolean lastIsBlue = false;
    private boolean patternSet = false;

    List<Blinker.Step> patternBlueOpen = new ArrayList<>();
    List<Blinker.Step> patternBlueDecay = new ArrayList<>();


    List<Blinker.Step> patternRedOpen = new ArrayList<>();
    List<Blinker.Step> patternRedDecay = new ArrayList<>();


    // Helper methods to create color int from RGB/ARGB values
    private static int rgb(int r, int g, int b) {
        return (r << 16) | (g << 8) | b;
    }

    private static int argb(int a, int r, int g, int b) {
        return (a << 24) | (r << 16) | (g << 8) | b;
    }



    public DefaultHubsCommand(HubsSubsystem hubsSubsystem, TrieurSubsystem trieurSubsystem) {
        this.hubsSubsystem = hubsSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(hubsSubsystem);
    }

    @Override
    public void initialize() {
        int patternDecaySteps = 5;
        int patternDecayMax = 20;
        int patternDecayMin = 0;
        int patternDecayDelta = patternDecayMax - patternDecayMin;
        int patternDecayStep = patternDecayDelta / patternDecaySteps;
        int patternDecayDurationMS = 200;
        int getPatternDecayStepDuration = patternDecayDurationMS / patternDecaySteps;

        for (int i = patternDecayMin; i < patternDecayMax +1; i += patternDecayStep) {
            patternBlueDecay.add(new Blinker.Step(rgb(i, i, 255), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternRedDecay.add(new Blinker.Step(rgb(255, i,  i), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternBlueOpen.add(new Blinker.Step(rgb(0, 255 - i, 255 - patternDecayDelta + i), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternRedOpen.add(new Blinker.Step(rgb(255 - patternDecayDelta + i, 255 - i, 0), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
        }
        for (int i = patternDecayMax; i > patternDecayMin -1; i -= patternDecayStep) {
            patternBlueDecay.add(new Blinker.Step(rgb(i, i, 255), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternRedDecay.add(new Blinker.Step(rgb(255, i, i), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternBlueOpen.add(new Blinker.Step(rgb(0, 255 - i, 255 - patternDecayDelta + i), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
            patternRedOpen.add(new Blinker.Step(rgb(255 - patternDecayDelta + i, 255 - i, 0), getPatternDecayStepDuration, TimeUnit.MILLISECONDS));
        }

        // Set initial pattern
        updatePattern(trieurSubsystem.isTrappeOpen(), hubsSubsystem.getOnBlueTeam());
    }
    @Override
    public void execute() {
        boolean currentTrappeOpen = trieurSubsystem.isTrappeOpen();
        boolean currentIsBlue = hubsSubsystem.getOnBlueTeam();

        // Only update pattern when trappe state or team color changes
        if (!patternSet || currentTrappeOpen != lastTrappeOpen || currentIsBlue != lastIsBlue) {
            updatePattern(currentTrappeOpen, currentIsBlue);
            lastTrappeOpen = currentTrappeOpen;
            lastIsBlue = currentIsBlue;
            patternSet = true;
        }
    }

    private void updatePattern(boolean isTrappeOpen, boolean isBlue) {
        if (isBlue) {
            if (isTrappeOpen) {
                hubsSubsystem.setPattern(patternBlueOpen);
            } else {
                hubsSubsystem.setPattern(patternBlueDecay);
            }
        } else {
            if (isTrappeOpen) {
                hubsSubsystem.setPattern(patternRedOpen);
            } else {
                hubsSubsystem.setPattern(patternRedDecay);
            }
        }
    }
}
