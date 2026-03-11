package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.ToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToPedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToFirstRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToSecondRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class TwoGateFromGoalGilles extends SequentialCommandGroup {


    public TwoGateFromGoalGilles(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {

        addCommands(
                new InitToPedroShooter(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                new ToSecondRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem,
                        new FollowPath(drivePedroSubsystem, builder -> builder
                                .addPath(new BezierCurve(
                                        drivePedroSubsystem::getPose,
                                        hubsSubsystem.getTeam().getRampPose()
                                                .withY(hubsSubsystem.getTeam().getRampPose().getY() + 3)
                                                .withX(hubsSubsystem.getTeam().getRampPose().getX() + (hubsSubsystem.getTeam().getRampPose().getX() > 72 ? -12 : 12)),
                                        hubsSubsystem.getTeam().getRampPose()))
                                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                        drivePedroSubsystem::getHeading,
                                        hubsSubsystem.getTeam().getRampPose().getHeading(),
                                        LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT)).build(),
                                AUTO_ROBOT_CONSTRAINTS, false),
                        LENGTH_X_ROW, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT, AUTO_ROBOT_CONSTRAINTS),

                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem.getTeam().getOpenRampPickPose(), hubsSubsystem.getTeam().getCloseShootPose(), GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem.getTeam().getOpenRampPickPose(), hubsSubsystem.getTeam().getCloseShootPose(), GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT),

                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem),

                new ToFirstRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem, LENGTH_X_ROW, LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT, AUTO_ROBOT_CONSTRAINTS),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())

        );
    }
}
