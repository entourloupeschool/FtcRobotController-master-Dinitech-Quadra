package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.LENGTH_X_ROW;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ToPlayerPickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.endsequence.RampEnd;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToQuickShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.ToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class PlayerPickFromAudience extends SequentialCommandGroup {

    public PlayerPickFromAudience(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, HubsSubsystem hubsSubsystem, double rowPower){
        Pose audienceShootPose = hubsSubsystem.getTeam().getAudienceShootPose();
        double audienceShootVelocity = hubsSubsystem.getTeam().getAudienceShootVelocity();

        addCommands(
                new InitToQuickShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, audienceShootPose, audienceShootVelocity, true, true),

                new ToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        hubsSubsystem.getTeam().getThirdRowPose(), audienceShootPose, audienceShootVelocity,
                        LENGTH_X_ROW, rowPower, true, true, true),

                new ToPlayerPickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem,
                        hubsSubsystem.getTeam().getPlayerPickPose(),
                        audienceShootPose, audienceShootVelocity),

                new RampEnd(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem.getTeam().getThirdRowPose())
        );
    }
}
