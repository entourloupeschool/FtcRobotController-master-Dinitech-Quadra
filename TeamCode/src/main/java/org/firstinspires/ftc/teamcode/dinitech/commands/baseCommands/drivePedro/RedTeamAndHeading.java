package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;


public class RedTeamAndHeading extends ConditionalCommand {
    /**
     * Creates a new ResetHeadingFCDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     */
    public RedTeamAndHeading(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
       super(
               new SwitchTeamAndHeading(drivePedroSubsystem, hubsSubsystem),
               new InstantCommand(),
               hubsSubsystem::getOnBlueTeam
       );

    }

}
