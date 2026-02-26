package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROULEAU_MOTOR_MAX_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TAPIS_MAX_SPEED;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class MaxPowerChargeur extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public MaxPowerChargeur(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setMotorPower(ROULEAU_MOTOR_MAX_POWER);
        chargeurSubsystem.setNormalizedSpeedTapis(TAPIS_MAX_SPEED);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
