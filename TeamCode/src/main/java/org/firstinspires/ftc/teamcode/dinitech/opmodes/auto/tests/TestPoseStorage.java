package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;


import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.RESET_POSE_BLUE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitCloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.AutoBase;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;

@Autonomous(name = "TestPoseStorage - Dinitech", group = "Test")

//@Disabled
public class TestPoseStorage extends AutoBase {

    @Override
    public void initialize(){
        super.initialize();

        hubsSubsystem.setTeam(TeamPoses.Team.BLUE);
        drivePedroSubsystem.prepAuto(RESET_POSE_BLUE);

        new SequentialCommandGroup(
                new InstantCommand(),
                new WaitToggleTrappe(trieurSubsystem),
                new WaitCommand(500),
                new WaitCloseTrappe(trieurSubsystem),
                new WaitCommand(500),
                new WaitOpenTrappe(trieurSubsystem)
        ).schedule();

    }



}
