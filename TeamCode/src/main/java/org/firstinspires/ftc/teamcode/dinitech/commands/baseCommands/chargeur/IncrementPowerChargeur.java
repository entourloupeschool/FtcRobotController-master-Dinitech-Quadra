package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class IncrementPowerChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;
    private final double powerIncrement;

    public IncrementPowerChargeur(ChargeurSubsystem chargeurSubsystem, double powerIncrement){
        this.chargeurSubsystem = chargeurSubsystem;
        this.powerIncrement = powerIncrement;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.incrementChargeurPower(powerIncrement);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
