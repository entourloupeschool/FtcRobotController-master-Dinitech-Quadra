package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.VoidEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.OnlyOpenGate;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToPedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class TwoGateFromGoalMR extends SequentialCommandGroup {


    public TwoGateFromGoalMR(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, HubsSubsystem hubsSubsystem) {

        addCommands(
                new InitToPedroShooter(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), hubsSubsystem.getTeam().getCloseShootVelocity()),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), hubsSubsystem.getTeam().getCloseShootPose(), LENGTH_X_ROW, AUTO_ROBOT_CONSTRAINTS, LINEAR_HEADING_INTERPOLATION_END_TIME, hubsSubsystem.getTeam().getCloseShootVelocity()),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem),

                new OnlyOpenGate(drivePedroSubsystem, hubsSubsystem.getTeam().getRampPose(), AUTO_ROBOT_CONSTRAINTS, LINEAR_HEADING_INTERPOLATION_END_TIME),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), hubsSubsystem.getTeam().getCloseShootPose(), LENGTH_X_ROW, AUTO_ROBOT_CONSTRAINTS, LINEAR_HEADING_INTERPOLATION_END_TIME, hubsSubsystem.getTeam().getCloseShootVelocity()),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())
        );
    }
}
