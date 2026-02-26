package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class IncrementSpeedTapis extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;
    private final double incr;

    public IncrementSpeedTapis(ChargeurSubsystem chargeurSubsystem, double incr){
        this.chargeurSubsystem = chargeurSubsystem;
        this.incr = incr;

        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.incrementSpeedTapis(3, incr);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
