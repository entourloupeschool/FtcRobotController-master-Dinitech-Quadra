package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class StopChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public StopChargeur(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setPower(0);
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}
