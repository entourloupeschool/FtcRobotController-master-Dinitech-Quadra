package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests.chars;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ADJUST_CONSTANT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_INCREMENT_SHOOTER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "ShooterChar - Dinitech", group = "Char")

public class ShooterChar extends GornetixRobotBase {
    // Gamepads
    private GamepadWrapper m_Driver, m_Operator;

    private GamepadSubsystem gamepadSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private double pShooter;
    private double iShooter;
    private double dShooter;
    private double fShooter;

    private double pShooterInit;
    private double iShooterInit;
    private double dShooterInit;
    private double fShooterInit;

    String[] paramNames = { "P", "I", "D", "F" };
    double[] paramValues = { pShooter, iShooter, dShooter, fShooter };

    private int selectedParamIndex = 0; // 0=P, 1=I, 2=D, 3=F

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
        register(shooterSubsystem);
        shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));

        PIDFCoefficients pidfCoeffsInit = shooterSubsystem.getPIDFVelocity();

        pShooterInit = pidfCoeffsInit.p;
        iShooterInit = pidfCoeffsInit.i;
        dShooterInit = pidfCoeffsInit.d;
        fShooterInit = pidfCoeffsInit.f;

        pShooter = pShooterInit;
        iShooter = iShooterInit;
        dShooter = dShooterInit;
        fShooter = fShooterInit;

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
            shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
        }

        if (leftTrigger > 0.01) {
            adjustSelectedParameter(-leftTrigger * leftTrigger * ADJUST_CONSTANT);
            shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
        }

        telemetryPIDF(telemetry);

        if (shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN)){
            shooterSubsystem.setPower(0);
        } else {
            shooterSubsystem.setPower(1);
        }

        super.run();
    }

    /**
     * Initialize GamepadEx wrappers for driver and operator.
     */
    private void setupGamePadsButtonBindings() {
        m_Driver = gamepadSubsystem.getDriver();
        m_Operator = gamepadSubsystem.getOperator();

        m_Operator.bump_right.whenPressed(() -> {
            shooterSubsystem.incrementVelocity(SPEED_INCREMENT_SHOOTER * 10);

        });
        m_Operator.bump_left.whenPressed(() -> {
            shooterSubsystem.incrementVelocity(-SPEED_INCREMENT_SHOOTER * 10);
        });
        m_Operator.triangle.whenPressed(() -> {
            shooterSubsystem.setVelocity(0);
        });

        m_Operator.dpad_down.whenPressed(
                () -> {
                    pShooter = pShooterInit;
                    iShooter = iShooterInit;
                    dShooter = dShooterInit;
                    fShooter = fShooterInit;

                    shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
                });

        m_Operator.dpad_up.whenPressed(
                () -> {
                    pShooter = 0;
                    iShooter = 0;
                    dShooter = 0;
                    fShooter = 0;

                    shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
                });

        // Parameter selection (left/right to navigate between P, I, D, F)
        m_Driver.square.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex - 1 + 7) % 7; // Wrap around: 0->3, 3->2, etc.
        });

        m_Driver.circle.whenPressed(() -> {
            selectedParamIndex = (selectedParamIndex + 1) % 7; // Wrap around: 0->1, 1->2, 2->3, 3->0
        });
        // Parameter adjustment (up/down to increase/decrease selected parameter)
        m_Driver.triangle.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(ADJUST_CONSTANT);
        }));
        m_Driver.triangle.whenReleased(() -> {
            shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
        });

        m_Driver.cross.whileHeld(new RunCommand(() -> {
            adjustSelectedParameter(-ADJUST_CONSTANT);
        }));
        m_Driver.cross.whenReleased(() -> {
            shooterSubsystem.setPIDFVelocity(pShooter, iShooter, dShooter, fShooter);
        });
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
                fShooter += delta;
                break;
        }
    }

    private void telemetryPIDF(Telemetry telemetry) {
        // Update paramValues with current PIDF values
        paramValues[0] = pShooter;
        paramValues[1] = iShooter;
        paramValues[2] = dShooter;
        paramValues[3] = fShooter;

        for (int i = 0; i < 4; i++) {
            String indicator = (i == selectedParamIndex) ? " <--" : "";
            telemetry.addData(paramNames[i] + indicator, String.format("%.6f", paramValues[i]));
        }
    }
}
