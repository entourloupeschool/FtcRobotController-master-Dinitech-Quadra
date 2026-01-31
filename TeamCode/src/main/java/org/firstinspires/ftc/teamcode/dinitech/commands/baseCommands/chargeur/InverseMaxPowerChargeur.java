package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_POWER;

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
        /**
         * Toggle the door state (open/close)
         */
        chargeurSubsystem.setChargeurPower(-CHARGEUR_MOTOR_POWER);
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}
