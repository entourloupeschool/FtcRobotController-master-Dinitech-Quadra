package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SAMPLE_SIZE_TEST;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.ContinuousRumbleCustom;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ContinuousUpdateColorSensorsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.UpdateColorSensorsDetections;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that manages the process of detecting an artifact and providing haptic feedback.
 * <p>
 * This command executes a two-stage process:
 * <ol>
 *     <li><b>Waiting for Proximity:</b> A {@link ParallelRaceGroup} runs a continuous low-intensity
 *     rumble on the gamepad while simultaneously waiting for the {@link TrieurSubsystem} to detect
 *     an artifact within its intake area. This provides feedback to the driver that the robot is
 *     ready to intake. This stage ends as soon as an artifact is detected.</li>
 *     <li><b>Confirming Detection:</b> Once an artifact is close, a second {@link ParallelRaceGroup}
 *     begins. It runs a higher-intensity rumble to confirm detection and simultaneously runs the
 *     {@link UpdateColorSensorsDetections} command to gather enough sensor data to accurately
 *     identify the artifact's color.</li>
 * </ol>
 */
public class DetectArtefactColor extends ParallelRaceGroup {

    /**
     * Creates a new DetectArtefact command.
     *
     * @param trieurSubsystem  The sorter subsystem, used for artifact and color detection.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public DetectArtefactColor(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        super(
                new ContinuousRumbleCustom(gamepadSubsystem, 3, 0.2),
                new ContinuousUpdateColorSensorsDetections(trieurSubsystem),
                new WaitUntilCommand(trieurSubsystem::isArtefactInTrieur)
        );
    }
}
