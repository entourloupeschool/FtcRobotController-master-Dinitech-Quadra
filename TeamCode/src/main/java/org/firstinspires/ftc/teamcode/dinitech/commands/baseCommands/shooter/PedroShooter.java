package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.ROTATED_BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.linearSpeedFromPedroRange;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PedroShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final HubsSubsystem hubsSubsystem;
    private Pose basketPose;


    /**
     * Creates a VisionShooter command that adjusts shooter speed based on AprilTag
     * range.
     *
     * @param shooterSubsystem The shooter subsystem
     * @param drivePedroSubsystem  The drive subsystem
     * @param hubsSubsystem Supplier for the goal pose
     */
    public PedroShooter(ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.hubsSubsystem = hubsSubsystem;
        this.basketPose = hubsSubsystem.getTeam().getBasketPose();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.PEDRO);
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE && drivePedroSubsystem.getDriveUsage() == DrivePedroSubsystem.DriveUsage.TELE){
            basketPose = ROTATED_BLUE_BASKET_POSE;
        } else {
            basketPose = hubsSubsystem.getTeam().getBasketPose();
        }
    }

    @Override
    public void execute() {
        shooterSubsystem.setVelocity(linearSpeedFromPedroRange(drivePedroSubsystem.getPose().distanceFrom(basketPose)));
    }
}
