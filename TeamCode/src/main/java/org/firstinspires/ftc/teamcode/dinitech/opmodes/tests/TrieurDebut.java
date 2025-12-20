package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SAMPLE_SIZE_TEST;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TeleDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinPrevious;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.StopMoulin;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.UpdateColorSensorsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ColoredArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "TrieurDebut - Dinitech", group = "Test")
public class TrieurDebut extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private TrieurSubsystem trieurSubsystem;
    private ChargeurSubsystem chargeurSubsystem;
    private VisionSubsystem visionSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
        register(gamepadSubsystem);

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        visionSubsystem.addAprilTagProcessor();
        visionSubsystem.buildVisionPortal();
        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
        register(trieurSubsystem);

        chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetry);
        register(chargeurSubsystem);

        setupGamePadsButtonBindings();

//                new MoulinCalibrate(trieurSubsystem);
        new MoulinCalibrationSequence(trieurSubsystem).schedule();

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
        GamepadWrapper m_Driver = gamepadSubsystem.driver;
        GamepadWrapper m_Operator = gamepadSubsystem.operator;

        // Driver controls
        m_Driver.circle.toggleWhenPressed(new AutomaticArtefactPickAway(trieurSubsystem,
                chargeurSubsystem, gamepadSubsystem));

        m_Driver.dpad_right.whenPressed(new MoulinNext(trieurSubsystem));
        m_Driver.dpad_left.whenPressed(new MoulinPrevious(trieurSubsystem));
        m_Driver.dpad_up.whenPressed(new MoulinRevolution(trieurSubsystem));
        m_Driver.dpad_down.whenPressed(new StopMoulin(trieurSubsystem));

        m_Driver.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
        m_Driver.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

        // Example trigger usage (you can uncomment and add commands as needed)
        new Trigger(trieurSubsystem::getIsFull)
                .whenActive(new ReadyMotif(trieurSubsystem, visionSubsystem,
                        gamepadSubsystem)
                );
    }
}
