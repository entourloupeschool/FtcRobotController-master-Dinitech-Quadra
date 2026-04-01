package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW_3RD;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getLinearInterpolationHeadingEndTimeFromRange;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.LookMotifPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.RaceLookMotifPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToPedroShootV2;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToGateToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ThreeRowsFromGoalWithGateOpen extends SequentialCommandGroup {

    public ThreeRowsFromGoalWithGateOpen(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, double rowPower){
        Pose closeShootPose = hubsSubsystem.getTeam().getCloseShootPose();
        addCommands(
                new InitToPedroShootV2(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem, closeShootPose),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem),

                new RaceLookMotifPath(drivePedroSubsystem, visionSubsystem, hubsSubsystem.getTeam().getLookMotifPose()),

                new ToRowToGateToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, hubsSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), closeShootPose, hubsSubsystem.getTeam().getRampPose(),
                        LENGTH_X_ROW, rowPower, false, 200),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, hubsSubsystem,
                        hubsSubsystem.getTeam().getThirdRowPose(), closeShootPose,
                        LENGTH_X_ROW_3RD, rowPower, false),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, hubsSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), closeShootPose,
                        LENGTH_X_ROW, rowPower, true),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())
        );
    }

}
