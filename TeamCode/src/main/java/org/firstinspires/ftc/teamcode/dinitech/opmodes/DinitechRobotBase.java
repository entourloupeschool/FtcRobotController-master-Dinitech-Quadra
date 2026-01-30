package org.firstinspires.ftc.teamcode.dinitech.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.List;

public class DinitechRobotBase extends CommandOpMode {
    // System
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    public TelemetryManager telemetryM;

    private final Globals.RunningAverage runningAverageFrequencies = new Globals.RunningAverage(10);

    // Subsystems
    public VisionSubsystem visionSubsystem;

    /**
     * Initialize all hardware and subsystems.
     */
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        timer.reset();
        telemetry.clearAll();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        hubs = hardwareMap.getAll(LynxModule.class);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule hub : hubs) {
            hub.abandonUnfinishedCommands();
            hub.clearBulkCache();
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    /**
     * Main OpMode loop. Handles voltage monitoring and cache clearing.
     */
    @Override
    public void run() {
        super.run();
        postCycle();
    }

    /**
     * Report loop time to telemetry.
     * Clear the bulk cache for all hubs.
     * Call telemetry update once per cycle
     */
    public void postCycle() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        runningAverageFrequencies.add(getFrequency());
//        telemetryM.addData("hz", freq);
        telemetryM.addData("average hz", getRunningAverageFreq());
//        telemetryM.addData("hz", freq);

        timer.reset();
        //
        // if (voltageSensor.getVoltage() < 8.5) {
        // throw new RuntimeException("voltage too low" + voltageSensor.getVoltage());
        // }

        telemetryM.update(telemetry);
    }

    public ElapsedTime getTimer() {
        return timer;
    }

    public double getElapsedTime() {
        return timer.seconds();
    }

    public double getFrequency() {
        return 1 / getElapsedTime();
    }

    public double getRunningAverageFreq(){
        return runningAverageFrequencies.getAverage();
    }

}
