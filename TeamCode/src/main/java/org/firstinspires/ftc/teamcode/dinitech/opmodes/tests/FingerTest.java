package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;


import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.WaitCloseFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.WaitOpenFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.WaitToggleFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.MoulinHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "FingerTest - Dinitech", group = "Test")
//@Disabled

public class FingerTest extends RobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private TrieurSubsystem trieurSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);


        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
        register(trieurSubsystem);

        setupGamePadsButtonBindings();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        super.run();
        telemetryM.addData("finger", trieurSubsystem.fingerAngle());
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        GamepadWrapper m_Driver = gamepadSubsystem.getDriver();
        GamepadWrapper m_Operator = gamepadSubsystem.getOperator();

        // Driver controls

        m_Driver.square.whenPressed(new WaitToggleFinger(trieurSubsystem));

        m_Driver.dpad_right.whenPressed(new WaitCloseFinger(trieurSubsystem));
        m_Driver.dpad_left.whenPressed(new WaitOpenFinger(trieurSubsystem));

        m_Driver.bump_right.whileHeld(new RunCommand(()->trieurSubsystem.incrCloseFinger(), trieurSubsystem));
        m_Driver.bump_left.whileHeld(new RunCommand(()->trieurSubsystem.incrOpenFinger(), trieurSubsystem));


    }
}
