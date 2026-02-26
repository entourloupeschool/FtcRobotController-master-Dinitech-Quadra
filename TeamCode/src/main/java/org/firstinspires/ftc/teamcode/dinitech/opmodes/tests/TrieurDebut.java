package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.StopMoulin;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "TrieurDebut - Dinitech", group = "Test")
public class TrieurDebut extends RobotBase {
    private GamepadSubsystem gamepadSubsystem;
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

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(visionSubsystem));

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
        register(trieurSubsystem);

        chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
        register(chargeurSubsystem);

        setupGamePadsButtonBindings();

//                new MoulinCalibrate(trieurSubsystem);
        new MoulinCalibrationSequence(trieurSubsystem).schedule();

    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        GamepadWrapper m_Driver = gamepadSubsystem.getDriver();
        GamepadWrapper m_Operator = gamepadSubsystem.getOperator();

        // Driver controls
        m_Driver.circle.toggleWhenPressed(new ModeRamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT));

        m_Driver.square.whenPressed(new ToggleChargeur(chargeurSubsystem));

        m_Driver.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
        m_Driver.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));
        m_Driver.dpad_up.whenPressed(new MoulinHighSpeedRevolution(trieurSubsystem));
        m_Driver.dpad_down.whenPressed(new StopMoulin(trieurSubsystem));

        m_Driver.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
        m_Driver.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

        m_Operator.dpad_left.whenPressed(new MoulinNextShoot(trieurSubsystem));
        m_Operator.dpad_right.whenPressed(new MoulinNextStorage(trieurSubsystem));

    }
}
