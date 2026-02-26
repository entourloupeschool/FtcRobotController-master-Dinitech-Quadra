package org.firstinspires.ftc.teamcode.dinitech.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class SetDefault extends CommandBase {
    private final SubsystemBase subsystemBase;
    private final CommandBase commandBase;

    public SetDefault(SubsystemBase subsystemBase, CommandBase commandBase){
        this.subsystemBase = subsystemBase;
        this.commandBase = commandBase;

        addRequirements(subsystemBase);
    }

    @Override
    public void initialize(){
        subsystemBase.setDefaultCommand(commandBase);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
