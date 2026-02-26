package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests.chars;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ADJUST_CONSTANT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEAING_POSE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.PedroAimLockedDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetHeadingFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SwitchTeamAndHeading;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixFullSystem;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.tele.GornetixTeleOp;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.Objects;

@TeleOp(name = "DrivingAimChar - Dinitech", group = "Char")
public class DrivingAimChar extends GornetixFullSystem {
    private double pDriveAim;
    private double iDriveAim;
    private double dDriveAim;
    private double fDriveAim;
    private double pDriveAimInit;
    private double iDriveAimInit;
    private double dDriveAimInit;
    private double fDriveAimInit;
    String[] paramNames = { "P", "I", "D", "F" };
    double[] paramValues = { pDriveAim, iDriveAim, dDriveAim, fDriveAim };
    private int selectedParamIndex = 0; // 0=P, 1=I, 2=D, 3=F

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        drivePedroSubsystem.getDrive().setPose(FIELD_CENTER_90HEAING_POSE);
        drivePedroSubsystem.setDefaultCommand(new PedroAimLockedDrive(drivePedroSubsystem, gamepadSubsystem, hubsSubsystem));

        m_Operator.start.whenPressed(new SwitchTeamAndHeading(drivePedroSubsystem, hubsSubsystem));

        setupGamePadsButtonBindings();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
        double rightTrigger = m_Operator.getRightTriggerValue();
        double leftTrigger = m_Operator.getLeftTriggerValue();

        if (rightTrigger > 0.01) {
            adjustSelectedParameter(rightTrigger * rightTrigger * ADJUST_CONSTANT);
//            drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim);
        }

        if (leftTrigger > 0.01) {
            adjustSelectedParameter(-leftTrigger * leftTrigger * ADJUST_CONSTANT);
//            drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim);
        }

        telemetryPIDF(telemetry);

        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        m_Operator.dpad_down.whenPressed(
                () -> {
                    pDriveAim = pDriveAimInit;
                    iDriveAim = iDriveAimInit;
                    dDriveAim = dDriveAimInit;
                    fDriveAim = fDriveAimInit;
//                    drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim);
                });

        m_Operator.dpad_up.whenPressed(
                () -> {
                    pDriveAim = 0;
                    iDriveAim = 0;
                    dDriveAim = 0;
                    fDriveAim = 0;
//                    drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim);
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
//        m_Driver.triangle.whenReleased(() -> drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim));

        m_Driver.cross.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(-ADJUST_CONSTANT);
        }));
//        m_Driver.cross.whenReleased(() -> drivePedroSubsystem.setAimControllerPIDF(pDriveAim, iDriveAim, dDriveAim, fDriveAim));
    }

    /**
     * Helper method to adjust the currently selected PIDF parameter.
     * 
     * @param delta The amount to increment/decrement
     */
    private void adjustSelectedParameter(double delta) {
        switch (selectedParamIndex) {
            case 0:
                pDriveAim += delta;
                break;
            case 1:
                iDriveAim += delta;
                break;
            case 2:
                dDriveAim += delta;
                break;
            case 3:
                fDriveAim += delta;
                break;
        }
    }

    private void telemetryPIDF(Telemetry telemetry) {
        // Update paramValues with current PIDF values
        paramValues[0] = pDriveAim;
        paramValues[1] = iDriveAim;
        paramValues[2] = dDriveAim;
        paramValues[3] = fDriveAim;

        for (int i = 0; i < 4; i++) {
            String indicator = (i == selectedParamIndex) ? " <--" : "";
            telemetry.addData(paramNames[i] + indicator, String.format("%.6f", paramValues[i]));
        }
    }
}
