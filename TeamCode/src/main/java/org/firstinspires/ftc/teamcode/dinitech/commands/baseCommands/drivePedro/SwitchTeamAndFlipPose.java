package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.hub.SwitchTeam;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;


public class SwitchTeamAndFlipPose extends SequentialCommandGroup {
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public SwitchTeamAndFlipPose(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
       addCommands(
               new SwitchTeam(hubsSubsystem),
               new FlipFieldCentricDrive(drivePedroSubsystem, BLUE_TEAM_HEADING)
       );

    }

}
