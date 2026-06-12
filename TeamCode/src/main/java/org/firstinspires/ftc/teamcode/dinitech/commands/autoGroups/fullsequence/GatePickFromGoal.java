package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;


import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW_3RD;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.ToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToPedroShootV2;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class GatePickFromGoal extends SequentialCommandGroup {
    public GatePickFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, HubsSubsystem hubsSubsystem, double rowPower) {
        Pose openRampPose = hubsSubsystem.getTeam().getOpenRampPose();
        Pose endRampPose = hubsSubsystem.getTeam().getEndRampPose();
        Pose closeShootPose = hubsSubsystem.getTeam().getCloseShootPose();
        double closeShootShooterVelocity = hubsSubsystem.getTeam().getCloseShootVelocity();

        addCommands(
                new InitToPedroShootV2(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        closeShootPose, closeShootShooterVelocity),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), closeShootPose,
                        closeShootShooterVelocity,
                        LENGTH_X_ROW_3RD, rowPower, false),

//                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem), // ALSO REQUIRES TRUE ON USE VISION SUBSYSTEM

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        openRampPose, endRampPose, closeShootPose, closeShootShooterVelocity, false),

                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), closeShootPose,
                        closeShootShooterVelocity,
                        LENGTH_X_ROW, 1, true),

                new RampEnd(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, openRampPose)
        );
    }

    public GatePickFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, double rowPower) {
        Pose openRampPose = hubsSubsystem.getTeam().getOpenRampPose();
        Pose endRampPose = hubsSubsystem.getTeam().getEndRampPose();
        Pose closeShootPose = hubsSubsystem.getTeam().getCloseShootPose();
        double closeShootShooterVelocity = hubsSubsystem.getTeam().getCloseShootVelocity();

        addCommands(
                new InitToPedroShootV2(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        closeShootPose, closeShootShooterVelocity),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), closeShootPose,
                        closeShootShooterVelocity,
                        LENGTH_X_ROW_3RD, rowPower, false),

//                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem), // ALSO REQUIRES TRUE ON USE VISION SUBSYSTEM

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, openRampPose, endRampPose, closeShootPose, closeShootShooterVelocity, false),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), closeShootPose,
                        closeShootShooterVelocity,
                        LENGTH_X_ROW, 1, true),

                new RampEnd(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, openRampPose)
        );
    }
}
