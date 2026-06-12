package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.FOLLOWER_T_POSITION_END;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getDynamicTEndFollower;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

import java.util.ArrayList;
import java.util.List;

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
 *
 * <p>Example usage with auto-configured target pose:
 * <pre>
 * new FollowPath(driveSubsystem, targetPose, 0.8, true);
 * </pre>
 */
public class FollowPath extends CommandBase {
    public final DrivePedroSubsystem drivePedroSubsystem;
    private final PathSupplier pathSupplier;
    private PathChain pathChain;
    private final double maxPower;
    private final boolean holdEnd;
    private final Runnable onExecuteCallback;
    private final boolean useDynamicTEnd;
    private final List<TemporalCallbackEntry> temporalCallbacks;
    private long startTimeMillis;

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
        this(drivePedroSubsystem, pathSupplier, maxPower, holdEnd, null);
    }

    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd, Runnable onExecuteCallback){
        this(drivePedroSubsystem, pathSupplier, maxPower, holdEnd, onExecuteCallback, true);
    }

    /**
     * Creates a FollowPath command using a PathSupplier (lambda-friendly), with an
     * optional callback called every scheduler loop while the path is active.
     *
     * @param drivePedroSubsystem The drive subsystem
     * @param pathSupplier Lambda that receives a PathBuilder and returns a PathChain
     * @param maxPower Maximum power (0-1)
     * @param holdEnd Whether to hold position at the end
     * @param onExecuteCallback Optional callback run in execute(); may be null
     */
    public FollowPath(DrivePedroSubsystem drivePedroSubsystem, PathSupplier pathSupplier, double maxPower, boolean holdEnd, Runnable onExecuteCallback, boolean useDynamicTEnd) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.pathSupplier = pathSupplier;
        this.pathChain = null;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        this.onExecuteCallback = onExecuteCallback;
        this.useDynamicTEnd = useDynamicTEnd;
        this.temporalCallbacks = new ArrayList<>();
        this.startTimeMillis = 0L;

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
        this.onExecuteCallback = null;
        this.useDynamicTEnd = true;
        this.temporalCallbacks = new ArrayList<>();
        this.startTimeMillis = 0L;

        addRequirements(drivePedroSubsystem);
    }

    public FollowPath addTemporalCallback(double millis, Runnable action) {
        temporalCallbacks.add(new TemporalCallbackEntry(millis, action));
        return this;
    }

    public FollowPath addTemporalCallbacks(Runnable action, double... millisValues) {
        for (double millisValue : millisValues) {
            addTemporalCallback(millisValue, action);
        }
        return this;
    }

    @Override
    public void initialize() {
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.AUTO);
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.NONE);
        startTimeMillis = System.currentTimeMillis();

        for (TemporalCallbackEntry temporalCallback : temporalCallbacks) {
            temporalCallback.hasTriggered = false;
        }


        // Build the path at initialization if using a supplier
        if (pathChain == null && pathSupplier != null) {
            pathChain = pathSupplier.build(drivePedroSubsystem.getPathBuilder());
        }

        drivePedroSubsystem.followPathChain(pathChain, maxPower, holdEnd);

        if (useDynamicTEnd) drivePedroSubsystem.setFollowerTEnd(getDynamicTEndFollower(pathChain.length()));
        else drivePedroSubsystem.setFollowerTEnd(FOLLOWER_T_POSITION_END);
    }

    @Override
    public void execute() {
        double elapsedMillis = System.currentTimeMillis() - startTimeMillis;
        for (TemporalCallbackEntry temporalCallback : temporalCallbacks) {
            if (!temporalCallback.hasTriggered && elapsedMillis >= temporalCallback.millis) {
                temporalCallback.hasTriggered = true;
                temporalCallback.action.run();
            }
        }

        if (onExecuteCallback != null) {
            onExecuteCallback.run();
        }
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return drivePedroSubsystem.isPathQuasiDone();
    }

    private static final class TemporalCallbackEntry {
        final double millis;
        final Runnable action;
        boolean hasTriggered;

        TemporalCallbackEntry(double millis, Runnable action) {
            this.millis = millis;
            this.action = action;
            this.hasTriggered = false;
        }
    }
}
