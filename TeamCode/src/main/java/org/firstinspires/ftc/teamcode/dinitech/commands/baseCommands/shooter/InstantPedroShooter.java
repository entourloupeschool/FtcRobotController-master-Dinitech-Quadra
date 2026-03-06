package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromPedroRange;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class InstantPedroShooter extends CommandBase {
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
    public InstantPedroShooter(ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.hubsSubsystem = hubsSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.PEDRO);

        double range = drivePedroSubsystem.getPose().distanceFrom(hubsSubsystem.getGoalPose());
        shooterSubsystem.setVelocity(linearSpeedFromPedroRange(range));
    }
}
