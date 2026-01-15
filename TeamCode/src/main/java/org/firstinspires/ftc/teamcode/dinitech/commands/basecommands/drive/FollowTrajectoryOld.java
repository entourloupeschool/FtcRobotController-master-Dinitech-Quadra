package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Set;

public class FollowTrajectoryOld extends CommandBase {
    private final Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    public FollowTrajectoryOld(Action action, Set<Subsystem> requirements) {
        this.action = action;
        this.requirements = requirements;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        
        // The run() method of an Action returns true if it is still running and false when it is finished.
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public Set<Subsystem> getRequirements(){
        return requirements;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
