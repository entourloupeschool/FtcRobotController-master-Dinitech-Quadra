package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_INCREMENT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CONTINUOUS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.IncrementPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "ChargeurDebut - Dinitech", group = "Test")
//@Disabled

public class ChargeurDebut extends RobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper m_Driver;
    private GamepadWrapper m_Operator;
    private TrieurSubsystem trieurSubsystem;
    private ChargeurSubsystem chargeurSubsystem;
    private VisionSubsystem visionSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);
        m_Driver = gamepadSubsystem.getDriver();
        m_Operator = gamepadSubsystem.getOperator();

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
        register(trieurSubsystem);

        chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
        register(chargeurSubsystem);

        setupGamePadsButtonBindings();

//                new MoulinCalibrate(trieurSubsystem);
//        new MoulinCalibrationSequence(trieurSubsystem).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double rightTriggerValue = m_Driver.getRightTriggerValue();
        double leftTriggerValue = m_Driver.getLeftTriggerValue();
        if (rightTriggerValue > 0.01) chargeurSubsystem.incrementTargetPower(rightTriggerValue * rightTriggerValue * 0.3);
        if (leftTriggerValue > 0.01) chargeurSubsystem.incrementTargetPower(-leftTriggerValue * leftTriggerValue * 0.3);

        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver.square.whenPressed(new MaxPowerChargeur(chargeurSubsystem));
        m_Driver.triangle.whenPressed(new StopChargeur(chargeurSubsystem));
    }
}
