package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.rouleau;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class IncrementPowerRouleau extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;
    private final double incr;

    public IncrementPowerRouleau(ChargeurSubsystem chargeurSubsystem, double incr){
        this.chargeurSubsystem = chargeurSubsystem;
        this.incr = incr;

        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.incrementMotorPower(incr);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
