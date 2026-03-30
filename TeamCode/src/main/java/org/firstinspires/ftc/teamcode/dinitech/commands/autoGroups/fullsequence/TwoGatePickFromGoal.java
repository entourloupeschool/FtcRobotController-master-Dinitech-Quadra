package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;


import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getLinearInterpolationHeadingEndTimeFromRange;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.LookMotifPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.ToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToQuickShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToGateToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class TwoGatePickFromGoal extends SequentialCommandGroup {
    public TwoGatePickFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, HubsSubsystem hubsSubsystem) {

        addCommands(
                new InitToQuickShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), hubsSubsystem.getTeam().getCloseShootVelocity()),

                new LookMotifPath(drivePedroSubsystem, hubsSubsystem.getTeam().getLookMotifPose()),

                new ToRowToGateToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), hubsSubsystem.getTeam().getCloseShootPose(), hubsSubsystem.getTeam().getRampPose(), LENGTH_X_ROW, 1,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose())), hubsSubsystem.getTeam().getCloseShootVelocity(), false,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose()))),

                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getOpenRampPickPose(), hubsSubsystem.getTeam().getCloseShootPose(), GATEPICK_POWER,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getOpenRampPickPose().distanceFrom(hubsSubsystem.getTeam().getCloseShootPose()))),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getOpenRampPickPose(), hubsSubsystem.getTeam().getCloseShootPose(),
                        GATEPICK_POWER,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getOpenRampPickPose()))),

                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), hubsSubsystem.getTeam().getCloseShootPose(),
                        LENGTH_X_ROW, 1,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getFirstRowPose())),
                        hubsSubsystem.getTeam().getCloseShootVelocity(), true,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getFirstRowPose()))),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())
        );
    }
}
