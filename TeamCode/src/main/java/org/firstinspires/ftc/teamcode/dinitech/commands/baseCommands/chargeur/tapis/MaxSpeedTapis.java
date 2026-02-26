package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TAPIS_MAX_SPEED;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class MaxSpeedTapis extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public MaxSpeedTapis(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setNormalizedSpeedTapis(TAPIS_MAX_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
