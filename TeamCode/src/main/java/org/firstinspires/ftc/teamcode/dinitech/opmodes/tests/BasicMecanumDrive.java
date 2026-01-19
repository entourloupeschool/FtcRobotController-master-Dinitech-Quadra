package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;

/*
Very basic usage of mecanum drive
 */
@TeleOp(name="BasicDrive - Dinitech", group="Test")
public class BasicMecanumDrive extends DinitechRobotBase {

    // Gamepads
    private DriveSubsystem driveSubsystem;
    private GamepadSubsystem gamepadSubsystem;
    public Pose2d beginPose;
    private VisionSubsystem visionSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();
        beginPose = new Pose2d(0, 0, 0);

        driveSubsystem = new DriveSubsystem(hardwareMap, beginPose, telemetry);
        register(driveSubsystem);

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);

        driveSubsystem.setDefaultCommand(new RobotCentricDrive(driveSubsystem, gamepadSubsystem));
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }

}
