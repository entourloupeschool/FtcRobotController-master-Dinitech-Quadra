package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.unchained;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUDIENCE_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToThirdRowToShoot extends ToRowToShoot {
    public ToThirdRowToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, Pose rowPose, Pose endPose, double lengthBackup, double rowPower){
        super(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, rowPose, endPose, lengthBackup, rowPower, LINEAR_HEADING_INTERPOLATION_END_TIME/1.3, AUDIENCE_AUTO_SHOOTER_VELOCITY);
    }
}
