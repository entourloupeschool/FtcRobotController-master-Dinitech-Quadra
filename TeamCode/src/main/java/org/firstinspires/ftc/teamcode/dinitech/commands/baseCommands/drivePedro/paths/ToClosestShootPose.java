package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_AUDIENCE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_SIDE_LENGTH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROBOT_LENGTH_INCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROBOT_WIDTH_INCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROTATED_BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getClosestVec2InLaunchZone;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
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
    protected Pose shootPose;

    public ToClosestShootPose(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem){
        addCommands(
                new InstantCommand(()->{
                    Pose currentPose = drivePedroSubsystem.getPose();
                    Pose basketPose = hubsSubsystem.getTeam().getBasketPose();

                    if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) {
                        currentPose = currentPose.rotate(BLUE_TEAM_HEADING, false);
                        basketPose = ROTATED_BLUE_BASKET_POSE;
                    }

                    Globals.Vec2 currentVec = new Globals.Vec2(currentPose.getX(), currentPose.getY());

                    Globals.Vec2 shootVec = getClosestVec2InLaunchZone(currentVec);

                    Globals.Vec2 basketVec = new Globals.Vec2(basketPose.getX(), basketPose.getY());
                    Globals.Vec2 shootToBasketVec = basketVec.subtract(shootVec);

                    double shootToBasketAngle = Math.atan2(shootToBasketVec.y, shootToBasketVec.x);

                    shootPose = new Pose(shootVec.x, shootVec.y, shootToBasketAngle);

                    if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) {
                        shootPose = shootPose.rotate(BLUE_TEAM_HEADING, true);
                    }
                }),
                OptimalPathV2.line(drivePedroSubsystem, shootPose, AUTO_ROBOT_CONSTRAINTS, true)
        );

    }


}