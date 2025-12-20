package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that shoots an artifact from a specific moulin position, using computer vision
 * to determine the appropriate shooter speed.
 * <p>
 * This class extends {@link ShootMoulinState} and overrides the default shooter command
 * with {@link VisionShooter}. Instead of using a fixed maximum speed, this command
 * adjusts the shooter's velocity based on data from the {@link VisionSubsystem},
 * ensuring the correct speed for the distance to the target.
 */
public class ShootMoulinStateVision extends ShootMoulinState {

    /**
     * Creates a new ShootMoulinStateVision command.
     *
     * @param trieurSubsystem       The sorter subsystem for moulin and trappe control.
     * @param shooterSubsystem      The shooter subsystem.
     * @param visionSubsystem       The vision subsystem for determining shooter speed.
     * @param moulinPositionToShoot The target shooting position (2, 4, or 6).
     */
    public ShootMoulinStateVision(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem,
                                  VisionSubsystem visionSubsystem, int moulinPositionToShoot) {
        super(trieurSubsystem, shooterSubsystem, moulinPositionToShoot,
                new VisionShooter(shooterSubsystem, visionSubsystem, false));
    }
}
