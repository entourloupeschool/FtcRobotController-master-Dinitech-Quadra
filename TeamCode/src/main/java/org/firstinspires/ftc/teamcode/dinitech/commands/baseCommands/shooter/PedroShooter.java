package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromPedroRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PedroShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final Supplier<Pose> goalPoseSupplier;


    /**
     * Creates a VisionShooter command that adjusts shooter speed based on AprilTag
     * range.
     *
     * @param shooterSubsystem The shooter subsystem
     * @param drivePedroSubsystem  The drive subsystem
     * @param goalPoseSupplier Supplier for the goal pose
     */
    public PedroShooter(ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, Supplier<Pose> goalPoseSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.goalPoseSupplier = goalPoseSupplier;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.PEDRO);
    }


    @Override
    public void execute() {
        double range = drivePedroSubsystem.getPose().distanceFrom(goalPoseSupplier.get());
        shooterSubsystem.setVelocity(linearSpeedFromPedroRange(range));
    }


}
