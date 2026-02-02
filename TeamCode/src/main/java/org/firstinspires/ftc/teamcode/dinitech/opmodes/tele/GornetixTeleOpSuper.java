package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_SMALL_TRIANGLE_SHOOT_POSE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.CancelFollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetPoseFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeShootSuper;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "GornetixTeleOpSuper - Dinitech", group = "TeleOp")
public class GornetixTeleOpSuper extends GornetixTeleOp {
    private GamepadWrapper m_Driver;
    private GamepadWrapper m_Operator;
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            m_Driver = gamepadSubsystem.getDriver();
            m_Operator = gamepadSubsystem.getOperator();

            setupGamePadsButtonBindings();
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

        //Overwrite m1 & m2
        m_Driver.dpad_down.whenPressed(new ResetPoseFCDrive(drivePedroSubsystem, getOnBlueTeam() ? CLOSE_SHOOT_BLUE_POSE : CLOSE_SHOOT_RED_POSE));
        m_Driver.dpad_up.whenPressed(new ResetPoseFCDrive(drivePedroSubsystem, getOnBlueTeam() ? BLUE_SMALL_TRIANGLE_SHOOT_POSE : RED_SMALL_TRIANGLE_SHOOT_POSE));
        m_Operator.start.whenPressed(new InstantCommand(() -> {
            setOnBlueTeam(!getOnBlueTeam());
        }));

        m_Driver.circle.whenPressed(new CancelFollowPath(drivePedroSubsystem));

        new Trigger(trieurSubsystem::getIsFull)
                .whenActive(new ModeShootSuper(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem, getOnBlueTeam()));


    }

}
