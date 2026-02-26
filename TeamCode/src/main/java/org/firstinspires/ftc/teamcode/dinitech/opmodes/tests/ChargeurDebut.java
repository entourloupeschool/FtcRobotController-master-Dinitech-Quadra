package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CHARGEUR_INCREMENT;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.rouleau.IncrementPowerRouleau;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.rouleau.MaxPowerRouleau;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.rouleau.StopRouleau;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis.IncrementSpeedTapis;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis.MaxSpeedTapis;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.tapis.StopTapis;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "ChargeurDebut - Dinitech", group = "Test")
public class ChargeurDebut extends RobotBase {
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
//        new MoulinCalibrationSequence(trieurSubsystem).schedule();

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
        m_Driver.square.whenPressed(new MaxPowerRouleau(chargeurSubsystem));
        m_Driver.cross.whenPressed(new MaxSpeedTapis(chargeurSubsystem));
        m_Driver.circle.whenPressed(new StopTapis(chargeurSubsystem));
        m_Driver.triangle.whenPressed(new StopRouleau(chargeurSubsystem));



        m_Operator.triangle.whenPressed(new IncrementPowerRouleau(chargeurSubsystem, CHARGEUR_INCREMENT));
        m_Operator.cross.whenPressed(new IncrementPowerRouleau(chargeurSubsystem, -CHARGEUR_INCREMENT));
        m_Operator.square.whenPressed(new IncrementSpeedTapis(chargeurSubsystem, CHARGEUR_INCREMENT));
        m_Operator.circle.whenPressed(new IncrementSpeedTapis(chargeurSubsystem, -CHARGEUR_INCREMENT));
    }
}
