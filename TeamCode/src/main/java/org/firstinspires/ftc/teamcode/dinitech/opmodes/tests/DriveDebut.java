package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/*
Very basic usage of mecanum drive
 */
@TeleOp(name="DriveDebut - Dinitech", group="Test")
public class DriveDebut extends GornetixRobotBase {

    // Gamepads
    private DrivePedroSubsystem drivePedroSubsystem;
    private GamepadSubsystem gamepadSubsystem;
    public Pose beginPose;
    private VisionSubsystem visionSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();
        beginPose = new Pose(0, 0, 0);

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, beginPose, telemetryM);
        register(drivePedroSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);


        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);

        drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }

}
