package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_STRENGTH_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME_VERY_SHORT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getBrakingStrengthScaleFromRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationHeadingEndTimeFromRange;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.BlueGoalAutoBase;

@Autonomous(name = "TestLateral - Dinitech", group = "Test")
//@Disabled
public class TestLateral extends BlueGoalAutoBase {

    @Override
    public void initialize() {
            super.initialize();

            new SequentialCommandGroup(
                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierLine(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getCloseShootPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getCloseShootPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getGoalInitPose().distanceFrom(hubsSubsystem.getTeam().getCloseShootPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getGoalInitPose().distanceFrom(hubsSubsystem.getTeam().getCloseShootPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierLine(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getFirstRowPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getFirstRowPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getFirstRowPose().distanceFrom(hubsSubsystem.getTeam().getCloseShootPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getFirstRowPose().distanceFrom(hubsSubsystem.getTeam().getCloseShootPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierCurve(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getSecondRowPose().withX(60),
                                    hubsSubsystem.getTeam().getSecondRowPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getSecondRowPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getFirstRowPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getFirstRowPose().distanceFrom(hubsSubsystem.getTeam().getSecondRowPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierCurve(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getThirdRowPose().withX(60),
                                    hubsSubsystem.getTeam().getThirdRowPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getThirdRowPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getSecondRowPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getSecondRowPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierLine(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getAudienceShootPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getAudienceShootPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getThirdRowPose().distanceFrom(hubsSubsystem.getTeam().getAudienceShootPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getThirdRowPose().distanceFrom(hubsSubsystem.getTeam().getAudienceShootPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierLine(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getThirdRowPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getThirdRowPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getThirdRowPose().distanceFrom(hubsSubsystem.getTeam().getAudienceShootPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getThirdRowPose().distanceFrom(hubsSubsystem.getTeam().getAudienceShootPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true),
                    new WaitCommand(2000),

                    new FollowPath(drivePedroSubsystem, builder -> builder
                            .addPath(new BezierCurve(
                                    drivePedroSubsystem::getPose,
                                    hubsSubsystem.getTeam().getCloseShootPose().withX(60),
                                    hubsSubsystem.getTeam().getCloseShootPose()))
                            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                    drivePedroSubsystem::getHeading,
                                    hubsSubsystem.getTeam().getCloseShootPose().getHeading(),
                                    getLinearInterpolationHeadingEndTimeFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))))
                            .setBrakingStrength(getBrakingStrengthScaleFromRange(hubsSubsystem.getTeam().getCloseShootPose().distanceFrom(hubsSubsystem.getTeam().getThirdRowPose()))).build(),
                            AUTO_ROBOT_CONSTRAINTS, true)
            ).schedule();
    }

    @Override
    public void run() {
            super.run();
    }


}
