package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW_3RD;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.LookMotifPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.RaceLookMotifPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToQuickShoot;
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
        addCommands(
                new InitToQuickShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getCloseShootPose(), hubsSubsystem.getTeam().getCloseShootVelocity(),
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getGoalInitPose()))),

                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true), trieurSubsystem),

                new LookMotifPath(drivePedroSubsystem, hubsSubsystem.getTeam().getLookMotifPose(),
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getLookMotifPose())),
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getLookMotifPose()))),

                new ToRowToGateToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getSecondRowPose(), hubsSubsystem.getTeam().getCloseShootPose(), hubsSubsystem.getTeam().getRampPose(),
                        LENGTH_X_ROW, rowPower,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose())), hubsSubsystem.getTeam().getCloseShootVelocity(), false,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose()))),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getThirdRowPose(), hubsSubsystem.getTeam().getCloseShootPose(),
                        LENGTH_X_ROW_3RD, rowPower,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose())), hubsSubsystem.getTeam().getCloseShootVelocity(), false,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getFirstRowPose(), hubsSubsystem.getTeam().getCloseShootPose(),
                        LENGTH_X_ROW, rowPower,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getFirstRowPose())), hubsSubsystem.getTeam().getCloseShootVelocity(), true,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getFirstRowPose()))),

                new RampEnd(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getRampPose())
        );
    }

}
