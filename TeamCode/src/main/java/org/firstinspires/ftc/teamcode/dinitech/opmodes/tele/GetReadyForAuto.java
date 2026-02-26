package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;

import java.util.Objects;

@TeleOp(name = "GetReadyForAuto - Dinitech", group = "TeleOp")
public class GetReadyForAuto extends GornetixTeleOp {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.getDrive().setPose(Objects.requireNonNullElseGet(PoseStorage.getLastPose(), () -> BEGIN_POSE));
            PoseStorage.clearLastPose();

            drivePedroSubsystem.dinitechPedroMecanumDrive.startTeleOpDrive(true);

            new SequentialCommandGroup(
                    new WaitCommand(100),
                    new MaxPowerChargeur(chargeurSubsystem),
                    new ModeRamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT)
            ).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }

}
