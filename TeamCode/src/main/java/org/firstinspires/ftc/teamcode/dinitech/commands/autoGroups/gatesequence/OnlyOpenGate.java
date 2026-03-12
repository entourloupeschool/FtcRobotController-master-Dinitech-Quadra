package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_FOR_3BALL;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class OnlyOpenGate extends SequentialCommandGroup {
    public OnlyOpenGate(DrivePedroSubsystem drivePedroSubsystem, Pose gatePickPose,  double gatePower, double endTime){
        addCommands(
            new FollowPath(drivePedroSubsystem, builder -> builder
                    .addPath(new BezierCurve(
                            drivePedroSubsystem::getPose,
                            gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -1.5*TILE_DIM : 1.5*TILE_DIM)),
                            gatePickPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            gatePickPose.getHeading(),
                            endTime)).build(),
                    AUTO_ROBOT_CONSTRAINTS, false),

            new FollowPath(drivePedroSubsystem, builder -> builder
                    .addPath(new BezierLine(
                            drivePedroSubsystem::getPose,
                            gatePickPose))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            Math.abs(gatePickPose.getHeading()) < Math.PI/2 ? 0 : Math.PI,
                            LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                    gatePower, true),
            new WaitCommand(5000),
            // Go to Shooting Pos
            new FollowPath(drivePedroSubsystem, builder -> builder
                    .addPath(new BezierLine(
                            drivePedroSubsystem::getPose,
                            gatePickPose.withX(gatePickPose.getX() + (gatePickPose.getX() > 72 ? -1.7*TILE_DIM : 1.7*TILE_DIM))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                            drivePedroSubsystem::getHeading,
                            gatePickPose.getHeading(),
                            LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                    AUTO_ROBOT_CONSTRAINTS, true)

        );
    }
}
