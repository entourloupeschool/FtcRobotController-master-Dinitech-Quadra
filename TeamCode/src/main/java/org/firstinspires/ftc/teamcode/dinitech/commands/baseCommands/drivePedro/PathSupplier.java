package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

/**
 * Functional interface for building paths with a PathBuilder.
 * This allows you to write paths inline using lambda expressions.
 *
 * Example usage:
 * <pre>
 * new FollowPath(driveSubsystem, builder -> builder
 *     .addPath(
 *         new BezierLine(
 *             new Point(startPose),
 *             new Point(endPose)
 *         )
 *     )
 *     .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
 *     .build(),
 *     0.8, true);
 * </pre>
 */
@FunctionalInterface
public interface PathSupplier {
    /**
     * Build a PathChain using the provided PathBuilder.
     *
     * @param builder The PathBuilder from the follower (starts at current robot pose)
     * @return The built PathChain
     */
    PathChain build(PathBuilder builder);
}
