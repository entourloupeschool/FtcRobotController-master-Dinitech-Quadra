package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

/**
 * A command-based subsystem for managing the driver and operator gamepads.
 * <p>
 * This class encapsulates the two physical gamepads into {@link GamepadWrapper} instances,
 * providing a clear distinction between the driver and operator roles. It simplifies access
 * to gamepad inputs and provides control over rumble effects for haptic feedback.
 *
 * @see GamepadWrapper
 * @see GamepadEx
 */
public class GamepadSubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    /** The wrapper for the driver's gamepad. */
    public final GamepadWrapper driver;
    /** The wrapper for the operator's gamepad. */
    public final GamepadWrapper operator;

    /**
     * Constructs a new GamepadSubsystem.
     *
     * @param gamepad1 The raw Gamepad object for the driver.
     * @param gamepad2 The raw Gamepad object for the operator.
     * @param telemetry The telemetry object for logging.
     */
    public GamepadSubsystem(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        gamepad1.reset();
        gamepad2.reset();

        // Initialize wrappers with distinct LED colors for easy identification
        driver = new GamepadWrapper(new GamepadEx(gamepad1), 1, 0.1, 0.1); // Red
        operator = new GamepadWrapper(new GamepadEx(gamepad2), 0.1, 0.1, 1); // Blue

        this.telemetry = telemetry;
    }

    /**
     * Gets the underlying {@link GamepadEx} object for the driver.
     * @return The driver's GamepadEx instance.
     */
    public GamepadEx getDriverEx(){
        return driver.getGamepadEx();
    }

    /**
     * Gets the underlying {@link GamepadEx} object for the operator.
     * @return The operator's GamepadEx instance.
     */
    public GamepadEx getOperatorEx(){
        return operator.getGamepadEx();
    }

    /**
     * Triggers a predefined rumble effect on the driver's gamepad.
     * @param rumbleEffectNumber The number of the rumble effect to play.
     */
    public void driverRumble(int rumbleEffectNumber){
        driver.rumble(rumbleEffectNumber);
    }

    /**
     * Triggers a predefined rumble effect on the operator's gamepad.
     * @param rumbleEffectNumber The number of the rumble effect to play.
     */
    public void operatorRumble(int rumbleEffectNumber){
        operator.rumble(rumbleEffectNumber);
    }

    /**
     * Triggers a custom rumble effect on one or both gamepads.
     *
     * @param customRumbleEffect The {@link Gamepad.RumbleEffect} to play.
     * @param gamepadNumber      The target gamepad(s): 1 for driver, 2 for operator, 3 for both.
     */
    public void customRumble(Gamepad.RumbleEffect customRumbleEffect, int gamepadNumber){
        switch (gamepadNumber){
            case 1:
                driver.runCustomRumble(customRumbleEffect);
                break;
            case 2:
                operator.runCustomRumble(customRumbleEffect);
                break;
            case 3:
                driver.runCustomRumble(customRumbleEffect);
                operator.runCustomRumble(customRumbleEffect);
                break;
        }
    }

    /**
     * Cancels any active rumble effect on one or both gamepads.
     *
     * @param gamepadNumber The target gamepad(s): 1 for driver, 2 for operator, 3 for both.
     */
    public void cancelRumble(int gamepadNumber){
        switch (gamepadNumber){
            case 1:
                driver.cancelCurrentRumble();
                break;
            case 2:
                operator.cancelCurrentRumble();
                break;
            case 3:
                driver.cancelCurrentRumble();
                operator.cancelCurrentRumble();
                break;
        }
    }
//
//    @Override
//    public void periodic() {
////        printGamepadWrapperTelemetry(driver, telemetry);
////        printGamepadWrapperTelemetry(operator, telemetry);
////        telemetry.addData("rightX", driver.getRightX()); // gives -1 when left and 1 when right
//
//    }

    private void printGamepadWrapperTelemetry(GamepadWrapper gamepadWrapper, final Telemetry telemetry){
        telemetry.addData("Gamepad " + 1, "(" + gamepadWrapper.getGamepadEx().gamepad.type().name() + ")");
        telemetry.addData("D-Pad Up " + 1, gamepadWrapper.dpad_up.get());
        telemetry.addData("D-Pad Down " + 1, gamepadWrapper.dpad_down.get());
        telemetry.addData("D-Pad Left " + 1, gamepadWrapper.dpad_left.get());
        telemetry.addData("D-Pad Right " + 1, gamepadWrapper.dpad_right.get());
    }

    private void printGamepadExTelemetry(GamepadEx gamepadEx, final Telemetry telemetry){
        telemetry.addData("PS Button", gamepadEx.gamepad.ps);
        telemetry.addData("Touchpad", gamepadEx.gamepad.touchpad);
        telemetry.addData("Finger 1 X", gamepadEx.gamepad.touchpad_finger_1_x);
        telemetry.addData("Finger 1 Y", gamepadEx.gamepad.touchpad_finger_1_y);
        telemetry.addData("Finger 2 X", gamepadEx.gamepad.touchpad_finger_2_x);
        telemetry.addData("Finger 2 Y", gamepadEx.gamepad.touchpad_finger_2_y);
    }
}
