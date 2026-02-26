package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.CancelFollowPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeShootSuper;

@TeleOp(name = "GornetixTeleOpSuper - Dinitech", group = "TeleOp")
public class GornetixTeleOpSuper extends GornetixTeleOp {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();
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

        m_Driver.circle.whenPressed(new CancelFollowPath(drivePedroSubsystem));

        new Trigger(trieurSubsystem::getIsFull)
                .whenActive(new ModeShootSuper(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, gamepadSubsystem, hubsSubsystem));


    }

}
