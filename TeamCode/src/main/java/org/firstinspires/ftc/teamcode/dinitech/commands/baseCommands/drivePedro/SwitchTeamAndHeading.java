package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_TEAM_HEADING;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;


public class SwitchTeamAndHeading extends SequentialCommandGroup {
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public SwitchTeamAndHeading(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
       addCommands(
               new InstantCommand(()-> hubsSubsystem.setOnBlueTeam(!hubsSubsystem.getOnBlueTeam())),
               new ConditionalCommand(
                       new SetPoseFCDrive(drivePedroSubsystem, new Pose(drivePedroSubsystem.getPose().getX(), drivePedroSubsystem.getPose().getY(), drivePedroSubsystem.getPose().getHeading() + BLUE_TEAM_HEADING)),
                       new SetPoseFCDrive(drivePedroSubsystem, new Pose(drivePedroSubsystem.getPose().getX(), drivePedroSubsystem.getPose().getY(), drivePedroSubsystem.getPose().getHeading() + RED_TEAM_HEADING)),
                       hubsSubsystem::getOnBlueTeam)
       );

    }

}
