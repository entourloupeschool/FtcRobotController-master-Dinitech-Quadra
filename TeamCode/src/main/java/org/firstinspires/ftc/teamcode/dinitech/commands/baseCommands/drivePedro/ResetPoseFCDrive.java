package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;


/**
 * A command for reseting the heading of the robot.
 *
 */
public class ResetPoseFCDrive extends SetPoseFCDrive {
    private final HubsSubsystem hubsSubsystem;
    public ResetPoseFCDrive(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
        super(drivePedroSubsystem, drivePedroSubsystem.getPose());
        this.hubsSubsystem = hubsSubsystem;
    }

    @Override
    public void initialize(){
        super.inputPose = hubsSubsystem.getTeam().getResetPose();
        super.initialize();
    }
}
