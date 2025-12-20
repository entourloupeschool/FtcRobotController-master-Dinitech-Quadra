package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RUMBLE_POWER;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A wrapper for the FTCLib {@link GamepadEx} class that simplifies access to gamepad controls.
 * <p>
 * This class provides easy-to-use {@link Button} and {@link Trigger} objects for all
 * standard gamepad inputs, as well as direct access to analog stick and trigger values.
 * It also includes features for controlling gamepad LED colors and rumble effects.
 *
 * @see GamepadEx
 * @see Button
 * @see Trigger
 */
public class GamepadWrapper {
    private final GamepadEx gamepadEx;
    
    // D-pad buttons
    public final Button dpad_up;
    public final Button dpad_down;
    public final Button dpad_left;
    public final Button dpad_right;
    
    // Face buttons
    public final Button cross;      // A button
    public final Button circle;     // B button
    public final Button square;     // X button
    public final Button triangle;   // Y button
    
    // Bumpers
    public final Button bump_left;
    public final Button bump_right;
    
    // Triggers
//    public final Button trig_left;
//    public final Button trig_right;
    
    // Additional buttons
    public final Button start;
    public final Button back;
    public final Button left_stick_button;
    public final Button right_stick_button;
    public final Trigger psButton;
    public final Trigger touchpadButton;
    public final Button m1Button;
    public final Button m2Button;

    private final Gamepad.RumbleEffect rumbleEffectUp, rumbleEffectDown, rumbleEffectSimple, rumbleEffectW;
    
    /**
     * Creates a GamepadWrapper for the given GamepadEx, and sets the LED color.
     * @param gamepadEx The GamepadEx to wrap.
     * @param r The normalized red value for the LED (0.0 to 1.0).
     * @param g The normalized green value for the LED (0.0 to 1.0).
     * @param b The normalized blue value for the LED (0.0 to 1.0).
     */
    public GamepadWrapper(GamepadEx gamepadEx, double r, double g, double b) {
        this.gamepadEx = gamepadEx;
        this.gamepadEx.gamepad.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);

        // Initialize D-pad buttons
        dpad_up = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_UP);
        dpad_down = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_DOWN);
        dpad_left = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_LEFT);
        dpad_right = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_RIGHT);
        
        // Initialize face buttons
        cross = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        circle = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        square = new GamepadButton(gamepadEx, GamepadKeys.Button.X);
        triangle = new GamepadButton(gamepadEx, GamepadKeys.Button.Y);
        
        // Initialize bumpers
        bump_left = new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);
        bump_right = new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);
        
        // Initialize triggers with threshold
//        trig_left = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > triggerThreshold);
//        trig_right = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > triggerThreshold);
        
        // Initialize additional buttons
        start = new GamepadButton(gamepadEx, GamepadKeys.Button.START);
        back = new GamepadButton(gamepadEx, GamepadKeys.Button.BACK);
        left_stick_button = new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_STICK_BUTTON);
        right_stick_button = new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        m1Button = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        m2Button = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        psButton = new Trigger(() -> gamepadEx.gamepad.ps);
        touchpadButton = new Trigger(() -> gamepadEx.gamepad.touchpad);

        rumbleEffectUp = new Gamepad.RumbleEffect.Builder()
                .addStep(RUMBLE_POWER/4, RUMBLE_POWER/4, 25)
                .addStep(0.0, 0.0, 5) 
                .addStep(RUMBLE_POWER/3, RUMBLE_POWER/3, 25)  
                .addStep(0.0, 0.0, 5)  
                .addStep(RUMBLE_POWER/2, RUMBLE_POWER/2, 25)  
                .addStep(0.0, 0.0, 5)  
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 100)  
                .build();

        rumbleEffectDown = new Gamepad.RumbleEffect.Builder()
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 50) 
                .addStep(0.0, 0.0, 5) 
                .addStep(RUMBLE_POWER/2, RUMBLE_POWER/2, 25)  
                .addStep(0.0, 0.0, 5)  
                .addStep(RUMBLE_POWER/3, RUMBLE_POWER/3, 25)  
                .addStep(0.0, 0.0, 5)  
                .addStep(RUMBLE_POWER/4, RUMBLE_POWER/4, 25)  
                .addStep(0.0, 0.0, 5)
                .build();

        rumbleEffectSimple = new Gamepad.RumbleEffect.Builder()
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 100)  //  Rumble both motor 100% for 100 mSec
                .build();

        rumbleEffectW = new Gamepad.RumbleEffect.Builder()
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 50)
                .addStep(0.0, 0.0, 25) 
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 100)  
                .addStep(0.0, 0.0, 25)  
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 100)  
                .addStep(0.0, 0.0, 25)  
                .addStep(RUMBLE_POWER, RUMBLE_POWER, 50)  
                .build();
                
    }

    /**
     * Gets the underlying GamepadEx object for direct access if needed.
     * @return The wrapped GamepadEx.
     */
    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }
    
    /**
     * Gets the current value of the left analog stick's X-axis.
     * @return The left stick X-axis value, from -1.0 (left) to 1.0 (right).
     */
    public double getLeftX() {
        return gamepadEx.getLeftX();
    }
    
    /**
     * Gets the current value of the left analog stick's Y-axis.
     * @return The left stick Y-axis value, from -1.0 (up) to 1.0 (down).
     */
    public double getLeftY() {
        return gamepadEx.getLeftY();
    }
    
    /**
     * Gets the current value of the right analog stick's X-axis.
     * @return The right stick X-axis value, from -1.0 (left) to 1.0 (right).
     */
    public double getRightX() {
        return gamepadEx.getRightX();
    }
    
    /**
     * Gets the current value of the right analog stick's Y-axis.
     * @return The right stick Y-axis value, from -1.0 (up) to 1.0 (down).
     */
    public double getRightY() {
        return gamepadEx.getRightY();
    }
    
    /**
     * Gets the current value of the left trigger.
     * @return The left trigger value, from 0.0 (unpressed) to 1.0 (fully pressed).
     */
    public double getLeftTriggerValue() {
        return gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }
    
    /**
     * Gets the current value of the right trigger.
     * @return The right trigger value, from 0.0 (unpressed) to 1.0 (fully pressed).
     */
    public double getRightTriggerValue() {
        return gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    /**
     * Gets the X position of the first finger on the touchpad.
     * @return The touchpad finger 1 X position, from -1.0 to 1.0.
     */
    public double getTouchpadFinger1X() {
        return gamepadEx.gamepad.touchpad_finger_1_x;
    }

    /**
     * Gets the Y position of the first finger on the touchpad.
     * @return The touchpad finger 1 Y position, from -1.0 to 1.0.
     */
    public double getTouchpadFinger1Y() {
        return gamepadEx.gamepad.touchpad_finger_1_y;
    }

    /**
     * Gets the X position of the second finger on the touchpad.
     * @return The touchpad finger 2 X position, from -1.0 to 1.0.
     */
    public double getTouchpadFinger2X() {
        return gamepadEx.gamepad.touchpad_finger_2_x;
    }

    /**
     * Gets the Y position of the second finger on the touchpad.
     * @return The touchpad finger 2 Y position, from -1.0 to 1.0.
     */
    public double getTouchpadFinger2Y() {
        return gamepadEx.gamepad.touchpad_finger_2_y;
    }

    /**
     * Triggers a predefined rumble effect on the gamepad.
     * <p>
     * The available effects are:
     * <ul>
     *     <li>1: Ramping up rumble</li>
     *     <li>2: Ramping down rumble</li>
     *     <li>3: Simple, short rumble</li>
     *     <li>4: "W" shaped rumble pattern</li>
     * </ul>
     * @param rumbleEffectNumber The number of the predefined rumble effect to play.
     */
    public void rumble(int rumbleEffectNumber){
        switch(rumbleEffectNumber){
            case 1:
                gamepadEx.gamepad.runRumbleEffect(rumbleEffectUp);
                break;
            case 2:
                gamepadEx.gamepad.runRumbleEffect(rumbleEffectDown);
                break;
            case 3:
                gamepadEx.gamepad.runRumbleEffect(rumbleEffectSimple);
                break;
            case 4:
                gamepadEx.gamepad.runRumbleEffect(rumbleEffectW);
                break;
        }
    }

    /**
     * Triggers a custom rumble effect on the gamepad.
     * @param customRumbleEffect The {@link Gamepad.RumbleEffect} to play.
     */
    public void runCustomRumble(Gamepad.RumbleEffect customRumbleEffect){
        gamepadEx.gamepad.runRumbleEffect(customRumbleEffect);
    }

    /**
     * Stops any currently active rumble effect on the gamepad.
     */
    public void cancelCurrentRumble(){
        gamepadEx.gamepad.stopRumble();
    }
}
