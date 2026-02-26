package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_TEAM_HEADING;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;


public class BlueTeamAndHeading extends ConditionalCommand {
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public BlueTeamAndHeading(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
       super(
               new InstantCommand(),
               new SwitchTeamAndHeading(drivePedroSubsystem, hubsSubsystem),
               hubsSubsystem::getOnBlueTeam
       );

    }

}
