package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_MOTOR_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;

public class StopDoubleServo extends CommandBase {
    private final ChargeurSubsystem chargeurSubsystem;

    public StopDoubleServo(ChargeurSubsystem chargeurSubsystem){
        this.chargeurSubsystem = chargeurSubsystem;
        addRequirements(chargeurSubsystem);
    }

    @Override
    public void initialize(){
        chargeurSubsystem.setPowerDoubleServo(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
