package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromPedroRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

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
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.PEDRO);
    }

    @Override
    public void execute() {
        double range = drivePedroSubsystem.getPose().distanceFrom(hubsSubsystem.getGoalPose());
        shooterSubsystem.setVelocity(linearSpeedFromPedroRange(range));
    }
}
