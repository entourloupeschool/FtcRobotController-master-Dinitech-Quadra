package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.hub;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class SwitchTeam extends CommandBase {
    private final HubsSubsystem hubsSubsystem;
    public SwitchTeam(HubsSubsystem hubsSubsystem) {
        this.hubsSubsystem = hubsSubsystem;
        addRequirements(hubsSubsystem);
    }

    @Override
    public void initialize() {
        if (hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE){
            hubsSubsystem.setTeam(HubsSubsystem.Team.RED);
        } else if (hubsSubsystem.getTeam() == HubsSubsystem.Team.RED){
            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
