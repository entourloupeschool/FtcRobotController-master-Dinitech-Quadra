package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class IncrementPowerChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;
    private double powerIncrement = 0;

    public IncrementPowerChargeur(ChargeurSubsystem chargeurSubsystem, double powerIncrement){
        this.chargeurSubsystem = chargeurSubsystem;
        this.powerIncrement = powerIncrement;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.incrementPower(powerIncrement);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
