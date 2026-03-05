package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_POWER_ROW_PICK_ARTEFACTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_RED_POSE;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ThreeRowsFromAudience extends SequentialCommandGroup {

    public ThreeRowsFromAudience(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem, double rowPower){
        addCommands(
                // Obelisk and MoulinCalibrate
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem,
                        hubsSubsystem),

                new SequentialCommandGroup(
                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                THIRD_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.3),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                SECOND_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                                FIRST_ROW_BLUE_POSE, LENGTH_X_ROW, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.7),

                        new ParallelCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.plus(new Pose(5, 0)) : RED_RAMP_POSE.plus(new Pose(- 5, 0))))
                                        .setLinearHeadingInterpolation(
                                                drivePedroSubsystem.getPose().getHeading(),
                                                hubsSubsystem.getTeam() == HubsSubsystem.Team.BLUE ? BLUE_RAMP_POSE.getHeading() : RED_RAMP_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new ParallelCommandGroup(
                                        new StopChargeur(chargeurSubsystem),
                                        new StopShooter(shooterSubsystem))))
        );


    }
}
