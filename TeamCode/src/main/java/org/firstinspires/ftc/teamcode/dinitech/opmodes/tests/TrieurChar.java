package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ADJUST_CONSTANT;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "TrieurChar - Dinitech", group = "Char")

public class TrieurChar extends DinitechRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;
    private GamepadSubsystem gamepadSubsystem;
    private TrieurSubsystem trieurSubsystem;

    private double pMoulin;
    private double iMoulin;
    private double dMoulin;
    private double fMoulin;

    private double pMoulinInit;
    private double iMoulinInit;
    private double dMoulinInit;
    private double fMoulinInit;
    String[] paramNames = { "P", "I", "D", "F" };
    double[] paramValues = { pMoulin, iMoulin, dMoulin, fMoulin };
    private int selectedParamIndex = 0; // 0=P, 1=I, 2=D, 3=F

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
        register(trieurSubsystem);

        PIDFCoefficients pidfCoeffsInit = trieurSubsystem.getPIDF();

        pMoulinInit = pidfCoeffsInit.p;
        iMoulinInit = pidfCoeffsInit.i;
        dMoulinInit = pidfCoeffsInit.d;
        fMoulinInit = pidfCoeffsInit.f;

        pMoulin = pMoulinInit;
        iMoulin = iMoulinInit;
        dMoulin = dMoulinInit;
        fMoulin = fMoulinInit;

        setupGamePadsButtonBindings();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double rightTrigger = m_Driver.getRightTriggerValue();
        double leftTrigger = m_Driver.getLeftTriggerValue();

        if (rightTrigger > 0.01) {
            adjustSelectedParameter(rightTrigger * rightTrigger * ADJUST_CONSTANT);
            trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin);
        }

        if (leftTrigger > 0.01) {
            adjustSelectedParameter(-leftTrigger * leftTrigger * ADJUST_CONSTANT);
            trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin);
        }

        telemetryPIDF(telemetry);

        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver = gamepadSubsystem.driver;
        m_Operator = gamepadSubsystem.operator;

        m_Operator.bump_right.whenPressed(new SequentialCommandGroup(
                new MoulinNextNext(trieurSubsystem),
                new WaitCommand(1),
                new MoulinNextNext(trieurSubsystem),
                new WaitCommand(1),
                new MoulinNext(trieurSubsystem)));
        m_Operator.bump_left.whenPressed(new MoulinRevolution(trieurSubsystem));

        m_Operator.dpad_down.whenPressed(
                () -> {
                    pMoulin = pMoulinInit;
                    iMoulin = iMoulinInit;
                    dMoulin = dMoulinInit;
                    fMoulin = fMoulinInit;
                    trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin);
                });

        m_Operator.dpad_up.whenPressed(
                () -> {
                    pMoulin = 0;
                    iMoulin = 0;
                    dMoulin = 0;
                    fMoulin = 0;
                    trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin);
                });

        // Parameter selection (left/right to navigate between P, I, D, F)
        m_Driver.square.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex - 1 + 4) % 4; // Wrap around: 0->3, 3->2, etc.
        });

        m_Driver.circle.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex + 1) % 4; // Wrap around: 0->1, 1->2, 2->3, 3->0
        });

        // Parameter adjustment (up/down to increase/decrease selected parameter)
        m_Driver.triangle.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(ADJUST_CONSTANT);
        }));
        m_Driver.triangle.whenReleased(() -> trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin));

        m_Driver.cross.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(-ADJUST_CONSTANT);
        }));
        m_Driver.cross.whenReleased(() -> trieurSubsystem.setPIDF(pMoulin, iMoulin, dMoulin, fMoulin));
    }

    /**
     * Helper method to adjust the currently selected PIDF parameter.
     * 
     * @param delta The amount to increment/decrement
     */
    private void adjustSelectedParameter(double delta) {
        switch (selectedParamIndex) {
            case 0:
                pMoulin += delta;
                break;
            case 1:
                iMoulin += delta;
                break;
            case 2:
                dMoulin += delta;
                break;
            case 3:
                fMoulin += delta;
                break;
        }
    }

    private void telemetryPIDF(Telemetry telemetry) {
        // Update paramValues with current PIDF values
        paramValues[0] = pMoulin;
        paramValues[1] = iMoulin;
        paramValues[2] = dMoulin;
        paramValues[3] = fMoulin;

        for (int i = 0; i < 4; i++) {
            String indicator = (i == selectedParamIndex) ? " <--" : "";
            telemetry.addData(paramNames[i] + indicator, String.format("%.6f", paramValues[i]));
        }
    }
}
