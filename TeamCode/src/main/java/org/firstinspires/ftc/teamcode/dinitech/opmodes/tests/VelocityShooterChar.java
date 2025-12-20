package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ADJUST_CONSTANT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_SHOOTER_SCALER;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterVelocitySubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "VelocityShooterChar - Dinitech", group = "Char")

public class VelocityShooterChar extends DinitechRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;
    private GamepadSubsystem gamepadSubsystem;
    private ShooterVelocitySubsystem shooterVelocitySubsystem;

    private double pShooter;
    private double iShooter;
    private double dShooter;
    private double kSShooter;
    private double kVShooter;
    private double kAShooter;

    String[] paramNames = { "P", "I", "D", "kS", "kV", "kA" };
    double[] paramValues = { pShooter, iShooter, dShooter, kSShooter, kVShooter, kAShooter };

    private int selectedParamIndex = 0; // 0=P, 1=I, 2=D

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);
        m_Driver = gamepadSubsystem.driver;
        m_Operator = gamepadSubsystem.operator;

        shooterVelocitySubsystem = new ShooterVelocitySubsystem(hardwareMap, telemetry);
        register(shooterVelocitySubsystem);


        shooterVelocitySubsystem.setPID(0, 0, 0);
        shooterVelocitySubsystem.setFF(0, 0, 0);

        PIDCoefficients pidCoeffsInit = shooterVelocitySubsystem.getPID();

        pShooter = pidCoeffsInit.p;
        iShooter = pidCoeffsInit.i;
        dShooter = pidCoeffsInit.d;

        double[] ffCoefs = shooterVelocitySubsystem.getFF();

        kSShooter = ffCoefs[0];
        kVShooter = ffCoefs[1];
        kAShooter = ffCoefs[2];

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
            shooterVelocitySubsystem.setPID(pShooter, iShooter, dShooter);
            shooterVelocitySubsystem.setFF(kSShooter, kVShooter, kAShooter);
        }

        if (leftTrigger > 0.01) {
            adjustSelectedParameter(-leftTrigger * leftTrigger * ADJUST_CONSTANT);
            shooterVelocitySubsystem.setPID(pShooter, iShooter, dShooter);
            shooterVelocitySubsystem.setFF(kSShooter, kVShooter, kAShooter);
        }

        telemetryPIDF(telemetry);

        super.run();
    }

    /**
     * Setup GamePads and Buttons and their associated commands.
     */
    private void setupGamePadsButtonBindings() {
        shooterVelocitySubsystem.setDefaultCommand(new RunCommand(() -> {
            double velocityIncrement = - m_Operator.getRightY();
            if (Math.abs(velocityIncrement) > 0.05){
                shooterVelocitySubsystem.incrementVelocity(velocityIncrement * velocityIncrement * velocityIncrement * TELE_SHOOTER_SCALER);
            }
        }, shooterVelocitySubsystem));

        m_Operator.triangle.whenPressed(new InstantCommand(() -> {
            shooterVelocitySubsystem.incrementVelocity(0.01);
        }));

        m_Operator.cross.whenPressed(new InstantCommand(() -> {
            shooterVelocitySubsystem.incrementVelocity(-0.01);
        }));

        // Parameter selection (left/right to navigate between P, I, D, F)
        m_Driver.square.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex - 1 + paramValues.length) % paramValues.length; // Wrap around: 0->3, 3->2, etc.
        });

        m_Driver.circle.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex + 1) % paramValues.length; // Wrap around: 0->1, 1->2, 2->3, 3->0
        });

        // Parameter adjustment (up/down to increase/decrease selected parameter)
        m_Driver.triangle.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(ADJUST_CONSTANT);
        }));
        m_Driver.triangle.whenReleased(this::setAll);

        m_Driver.cross.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(-ADJUST_CONSTANT);
        }));
        m_Driver.cross.whenReleased(this::setAll);
    }

    /**
     * Helper method to adjust the currently selected PIDF parameter.
     * 
     * @param delta The amount to increment/decrement
     */
    private void adjustSelectedParameter(double delta) {
        switch (selectedParamIndex) {
            case 0:
                pShooter += delta;
                break;
            case 1:
                iShooter += delta;
                break;
            case 2:
                dShooter += delta;
                break;
            case 3:
                kSShooter += delta;
                break;
            case 4:
                kVShooter += delta;
                break;
            case 5:
                kAShooter += delta;
                break;
        }
    }

    private void telemetryPIDF(Telemetry telemetry) {
        // Update paramValues with current PIDF values
        paramValues[0] = pShooter;
        paramValues[1] = iShooter;
        paramValues[2] = dShooter;
        paramValues[3] = kSShooter;
        paramValues[4] = kVShooter;
        paramValues[5] = kAShooter;


        for (int i = 0; i < paramValues.length; i++) {
            String indicator = (i == selectedParamIndex) ? " <--" : "";
            telemetry.addData(paramNames[i] + indicator, String.format("%.6f", paramValues[i]));
        }
    }

    private void setAll(){
        shooterVelocitySubsystem.setPID(pShooter, iShooter, dShooter);
        shooterVelocitySubsystem.setFF(kSShooter, kVShooter, kAShooter);
    }
}
