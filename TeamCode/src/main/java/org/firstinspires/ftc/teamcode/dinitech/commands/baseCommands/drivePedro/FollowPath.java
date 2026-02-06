package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

/**
 * Command to follow a path using PedroPathing.
 *
 * <p>Example usage with PathSupplier (recommended for inline paths):
 * <pre>
 * new FollowPath(driveSubsystem, builder -> builder
 *     .addPath(new BezierLine(
 *         new Point(0, 0, Point.CARTESIAN),
 *         new Point(24, 24, Point.CARTESIAN)
 *     ))
 *     .setLinearHeadingInterpolation(0, Math.PI / 2)
 *     .build(),
 *     0.8, true);
 * </pre>
 *
 * <p>Example usage with pre-built PathChain:
 * <pre>
 * PathChain myPath = driveSubsystem.getDrive().getPathBuilder()
 *     .addPath(...)
 *     .build();
 * new FollowPath(driveSubsystem, myPath, 0.8, true);
 * </pre>
 */
public class FollowPath extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final PathSupplier pathSupplier;
    private PathChain pathChain;
    private double maxPower;
    private final boolean holdEnd;

    /**
     * Creates a FollowPath command using a PathSupplier (lambda-friendly).
     * The path is built at initialization time using the current robot pose.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathSupplier Lambda that receives a PathBuilder and returns a PathChain
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.pathSupplier = pathSupplier;
        this.pathChain = null;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;

        addRequirements(drivePedroSubsystem);
    }

    /**
     * Creates a FollowPath command with a pre-built PathChain.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathChain The pre-built PathChain to follow
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathChain pathChain, double maxPower, boolean holdEnd) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.pathSupplier = null;
        this.pathChain = pathChain;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;

        addRequirements(drivePedroSubsystem);
    }


    @Override
    public void initialize() {
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.NONE);

        // Build the path at initialization if using a supplier
        if (pathChain == null && pathSupplier != null) {
            pathChain = pathSupplier.build(drivePedroSubsystem.getDrive().getPathBuilder());
        }

        drivePedroSubsystem.followPathChain(pathChain, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return drivePedroSubsystem.isPathQuasiDone();
    }
}
