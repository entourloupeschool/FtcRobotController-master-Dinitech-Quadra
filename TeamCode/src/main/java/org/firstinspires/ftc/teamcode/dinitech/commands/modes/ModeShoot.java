package org.firstinspires.ftc.teamcode.dinitech.commands.modes;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TeleDriveLocked;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeShoot extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param driveSubsystem    The drive subsystem for driving the robot.
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param shooterSubsystem  The shooter subsystem for running the shooter motor.
     * @param visionSubsystem the vision subsystem
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeShoot(DriveSubsystem driveSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem,
                     VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> {visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));}
                        ),
                        new StopChargeur(chargeurSubsystem)),
                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                new TeleDriveLocked(driveSubsystem, visionSubsystem, gamepadSubsystem),
                new VisionShooter(shooterSubsystem, visionSubsystem, true)
        );
    }
}
