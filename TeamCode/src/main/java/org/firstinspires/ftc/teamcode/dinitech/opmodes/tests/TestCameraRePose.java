package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AUTO_ROBOT_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.VISION_RE_POSE_AT_RANGE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.CancelFollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetPoseFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "CameraRePoseTest - Dinitech", group = "Test")
public class TestCameraRePose extends GornetixRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper driver;
    private Gamepad.RumbleEffect customRumbleEffect;
    private VisionSubsystem visionSubsystem;
    private DrivePedroSubsystem drivePedroSubsystem;

    private final Pose initPose = new Pose(48, 96, Math.PI/2);
    private final Pose closePose = new Pose(27.2, 116, 3*Math.PI/4);
    private final Pose farPose = new Pose(110, 34, 3*Math.PI/4);

    private final Pose controlPointGoClose = new Pose(initPose.getX(),closePose.getY(), Math.PI);
    private final Pose controlPointFromClose = new Pose(closePose.getX(), closePose.getY(), Math.PI);
    private final Pose controlPointGoFar = new Pose(farPose.getX(),initPose.getY(), Math.PI);
    private final Pose controlPointFromFar = new Pose(initPose.getX(), farPose.getY(), Math.PI);


    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);
        driver = gamepadSubsystem.getDriver();
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 50)
                .build();

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, initPose.withHeading((double) 3 /4*Math.PI), telemetryM);
        register(drivePedroSubsystem);
        drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem));

        driver.cross.toggleWhenPressed(new SequentialCommandGroup(
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierCurve(
                                drivePedroSubsystem::getPose,
                                controlPointGoClose,
                                closePose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                closePose.getHeading(),
                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                        AUTO_ROBOT_CONSTRAINTS, false),

                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierCurve(
                                drivePedroSubsystem::getPose,
                                controlPointFromClose,
                                initPose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                Math.PI,
                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                        AUTO_ROBOT_CONSTRAINTS, false),
                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierCurve(
                                drivePedroSubsystem::getPose,
                                controlPointGoFar,
                                farPose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                farPose.getHeading(),
                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                        AUTO_ROBOT_CONSTRAINTS, false),

                new FollowPath(drivePedroSubsystem, builder -> builder
                        .addPath(new BezierCurve(
                                drivePedroSubsystem::getPose,
                                controlPointFromFar,
                                initPose))
                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                                drivePedroSubsystem::getHeading,
                                initPose.getHeading(),
                                LINEAR_HEADING_INTERPOLATION_END_TIME)).build(),
                        AUTO_ROBOT_CONSTRAINTS, false)));

        driver.triangle.whenPressed(new CancelFollowPath(drivePedroSubsystem));
        driver.square.toggleWhenPressed(new InstantCommand(() -> drivePedroSubsystem.getDrive().getFollower().pausePathFollowing()), new InstantCommand(() -> drivePedroSubsystem.getDrive().getFollower().resumePathFollowing()));
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();

//            if (visionSubsystem.getHasCurrentAprilTagDetections()){
//                Double range = visionSubsystem.getRangeToAprilTag();
//                if (range != null){
//                    if (range < VISION_RE_POSE_AT_RANGE){
//                        Pose visionPose = visionSubsystem.getLatestRobotPoseEstimationFromAT();
//                        drivePedroSubsystem.getDrive().setPose(visionPose);
//                        gamepadSubsystem.customRumble(customRumbleEffect, 1, true);
//                    }
//                }
//            }
    }

}
