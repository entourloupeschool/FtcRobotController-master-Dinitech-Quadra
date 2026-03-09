package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.hub;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

public class SwitchTeam extends CommandBase {
    private final HubsSubsystem hubsSubsystem;
    public SwitchTeam(HubsSubsystem hubsSubsystem) {
        this.hubsSubsystem = hubsSubsystem;
        addRequirements(hubsSubsystem);
    }

    @Override
    public void initialize() {
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE){
            hubsSubsystem.setTeam(TeamPoses.Team.RED);
        } else if (hubsSubsystem.getTeam() == TeamPoses.Team.RED){
            hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
