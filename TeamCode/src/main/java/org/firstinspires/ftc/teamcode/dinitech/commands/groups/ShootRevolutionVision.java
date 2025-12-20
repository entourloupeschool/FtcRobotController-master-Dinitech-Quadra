package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that performs a full revolution to shoot all artifacts, using computer vision
 * to determine the appropriate shooter speed.
 * <p>
 * This class extends {@link ShootRevolution} and overrides the default shooter command
 * with {@link VisionShooter}. Instead of using a fixed maximum speed, this command
 * adjusts the shooter's velocity based on data from the {@link VisionSubsystem},
 * likely calculating the required speed based on the distance to the target.
 */
public class ShootRevolutionVision extends ShootRevolution {

    /**
     * Creates a new ShootRevolutionVision command.
     *
     * @param trieurSubsystem  The sorter subsystem for moulin and trappe control.
     * @param shooterSubsystem The shooter subsystem.
     * @param visionSubsystem  The vision subsystem for determining shooter speed.
     */
    public ShootRevolutionVision(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem){
        super(trieurSubsystem, shooterSubsystem, new VisionShooter(shooterSubsystem, visionSubsystem, false));
    }
}
