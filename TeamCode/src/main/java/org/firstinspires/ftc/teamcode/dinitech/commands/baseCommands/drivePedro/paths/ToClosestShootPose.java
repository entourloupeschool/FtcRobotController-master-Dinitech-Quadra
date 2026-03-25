package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROTATED_BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getClosestVec2InLaunchZone;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

/**
 * Path builder tuned to maximize tangent heading travel while minimizing rotation demand.
 *
 * <p>Rules:
 * <ol>
 *   <li>If range is short, use pure linear heading interpolation.</li>
 *   <li>If range is long enough, split heading into:
 *     rotate to tangent -> tangent/reverse tangent -> rotate to final heading.</li>
 *   <li>Pick tangent or reverse-tangent based on the smallest total heading change.</li>
 * </ol>
 */
public class ToClosestShootPose extends SequentialCommandGroup {
    public ToClosestShootPose(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem){
        addCommands(
                new OptimalPath(
                        drivePedroSubsystem,
                        OptimalPath.createLineSupplier(
                                drivePedroSubsystem,
                                (currentPose, currentHeading) -> computeShootPose(currentPose, hubsSubsystem)),
                        AUTO_ROBOT_CONSTRAINTS,
                        true)

        );

    }

    private static Pose computeShootPose(Pose currentPose, HubsSubsystem hubsSubsystem) {
        Pose basketPose = hubsSubsystem.getTeam().getBasketPose();
        if (basketPose == null) {
            return currentPose;
        }

        Pose workingPose = currentPose;
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) {
            workingPose = currentPose.rotate(BLUE_TEAM_HEADING, false);
        }

        Globals.Vec2 currentVec = new Globals.Vec2(workingPose.getX(), workingPose.getY());
        Globals.Vec2 shootVec = getClosestVec2InLaunchZone(currentVec);

        Globals.Vec2 basketVec = new Globals.Vec2(basketPose.getX(), basketPose.getY());
        Globals.Vec2 shootToBasketVec = basketVec.subtract(shootVec);
        double shootToBasketAngle = Math.atan2(shootToBasketVec.y, shootToBasketVec.x);

        Pose shootPose = new Pose(shootVec.x, shootVec.y, shootToBasketAngle);
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) {
            shootPose = shootPose.rotate(BLUE_TEAM_HEADING, true);
        }

        return shootPose;
    }


}