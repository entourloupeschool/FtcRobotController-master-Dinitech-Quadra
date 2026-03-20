package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence.mr;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_WALL_PICK;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ToWallToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToQuickShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class WallPickFromAudience extends SequentialCommandGroup {

    public WallPickFromAudience(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, double rowPower){
        addCommands(
                new InitToQuickShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem.getTeam().getAudienceShootPose(), hubsSubsystem.getTeam().getAudienceShootVelocity(),
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getAudienceInitPose().distanceFrom(hubsSubsystem.getTeam().getAudienceShootPose()))),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem,
                        hubsSubsystem.getTeam().getThirdRowPose(), hubsSubsystem.getTeam().getAudienceShootPose(),
                        LENGTH_X_ROW, rowPower,
                        getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getAudienceShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose())),
                        hubsSubsystem.getTeam().getAudienceShootVelocity(), true,
                        getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getAudienceShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))),

                new ToWallToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, hubsSubsystem.getTeam().getWallPickPose(), hubsSubsystem.getTeam().getAudienceShootPose(), LENGTH_WALL_PICK, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME, AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY)
        );
    }
}
