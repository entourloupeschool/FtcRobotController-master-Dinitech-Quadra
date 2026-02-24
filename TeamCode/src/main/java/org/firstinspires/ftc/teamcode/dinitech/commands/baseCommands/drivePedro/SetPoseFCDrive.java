package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;


/**
 * A command for reseting the heading of the robot.
 *
 */
public class ResetPoseFCDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final Pose inputPose;
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public ResetPoseFCDrive(DrivePedroSubsystem drivePedroSubsystem, Pose inputPose) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.inputPose = inputPose;
        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize(){
        drivePedroSubsystem.getDrive().setPose(inputPose);
        drivePedroSubsystem.setDriverInputPose(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
