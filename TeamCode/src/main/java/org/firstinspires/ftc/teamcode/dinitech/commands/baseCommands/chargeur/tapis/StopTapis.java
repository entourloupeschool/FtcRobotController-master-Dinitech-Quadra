package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class StopTapis extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public StopTapis(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setNormalizedSpeedTapis(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
