package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.rouleau;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROULEAU_MOTOR_MAX_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TAPIS_MAX_SPEED;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class MaxPowerRouleau extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public MaxPowerRouleau(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setMotorPower(ROULEAU_MOTOR_MAX_POWER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
