package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.THIRD_ROW_BLUE_POSE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.unchained.ToFirstRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.unchained.ToSecondRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.unchained.ToThirdRowToShoot;
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

public class BlueThreeRowsFromGoal extends SequentialCommandGroup {

    public BlueThreeRowsFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, GamepadSubsystem gamepadSubsystem){
        addCommands(
                // Obelisk and MoulinCalibrate
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                new SequentialCommandGroup(
                        new ToFirstRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                FIRST_ROW_BLUE_POSE, CLOSE_SHOOT_BLUE_POSE, LENGTH_X_ROW, AUTO_ROBOT_CONSTRAINTS),
                        new ToSecondRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                SECOND_ROW_BLUE_POSE, CLOSE_SHOOT_BLUE_POSE, LENGTH_X_ROW, AUTO_ROBOT_CONSTRAINTS),
                        new ToThirdRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                THIRD_ROW_BLUE_POSE, BLUE_AUDIENCE_SHOOT_POSE, LENGTH_X_ROW, AUTO_ROBOT_CONSTRAINTS),


                        new ParallelCommandGroup(
                                new FollowPath(drivePedroSubsystem, builder -> builder
                                        .addPath(new BezierLine(
                                                drivePedroSubsystem::getPose,
                                                BLUE_RAMP_POSE.withX(BLUE_RAMP_POSE.getX() - 5)))
                                        .setLinearHeadingInterpolation(
                                                drivePedroSubsystem.getPose().getHeading(),
                                                BLUE_RAMP_POSE.getHeading(),
                                                LINEAR_HEADING_INTERPOLATION_END_TIME).build(),
                                        AUTO_ROBOT_CONSTRAINTS, true),
                                new ParallelCommandGroup(
                                        new StopChargeur(chargeurSubsystem),
                                        new StopShooter(shooterSubsystem))))
        );
    }

}
