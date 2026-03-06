package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.ToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToFirstRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToSecondRowOpenGateToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToSecondRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class TwoGateFromGoal extends SequentialCommandGroup {


    public TwoGateFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {

        addCommands(
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                new ToSecondRowOpenGateToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem, LENGTH_X_ROW, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem.getTeam().getRampPose(), hubsSubsystem.getTeam().getEndRampPose(), hubsSubsystem.getTeam().getCloseShootPose(), GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true)),
                
                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem.getTeam().getRampPose(), hubsSubsystem.getTeam().getEndRampPose(), hubsSubsystem.getTeam().getCloseShootPose(), GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new ToFirstRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem, LENGTH_X_ROW, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())

        );

    }



}
