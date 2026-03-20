package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROULEAU_MOTOR_MAX_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TAPIS_MAX_SPEED;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class InverseMaxPowerChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public InverseMaxPowerChargeur(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setTargetPower(-ROULEAU_MOTOR_MAX_POWER/1.5);
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}
