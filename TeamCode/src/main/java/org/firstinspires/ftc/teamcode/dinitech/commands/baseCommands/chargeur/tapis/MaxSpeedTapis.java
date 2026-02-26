package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class MaxPowerDoubleServo extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public MaxPowerDoubleServo(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setNormalizedSpeedTapis(CHARGEUR_MOTOR_POWER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
