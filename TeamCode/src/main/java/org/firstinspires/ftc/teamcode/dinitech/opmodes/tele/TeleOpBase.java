package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIELD_CENTER_90HEADING_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LONG_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MID_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SHOOT_REVOLUTION_THEN_WAIT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.MaxPowerDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetHeadingFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SwitchAimLockType;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SwitchTeamAndFlipPose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SwitchUsageStateShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OptimizedUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootGreen;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootHighSpeedRevolutionTeleOp;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootPurple;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageTeleOp;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeShootTeleOp;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.Gornetix;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;

import java.util.Objects;

public class GornetixTeleOpBase extends Gornetix {
    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.getDrive().setPose(Objects.requireNonNullElseGet(PoseStorage.getLastPose(), () -> FIELD_CENTER_90HEADING_POSE));
            PoseStorage.clearLastPose();

            drivePedroSubsystem.dinitechPedroMecanumDrive.startTeleOpDrive(true);

            drivePedroSubsystem.setLastTeleDriverPowerScale(1);

            visionSubsystem.setDefaultCommand(new OptimizedUpdatesAprilTagsDetections(visionSubsystem, drivePedroSubsystem, trieurSubsystem, shooterSubsystem));

            setupGamePadsButtonBindings();

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
        gamepadSubsystem.setDefaultCommand(new DefaultGamepadCommand(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, gamepadSubsystem));
        visionSubsystem.setDefaultCommand(new OptimizedUpdatesAprilTagsDetections(visionSubsystem, drivePedroSubsystem, trieurSubsystem, shooterSubsystem));
        drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem));
        shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));

        // Full stop robot
        m_Driver.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));
        m_Operator.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));

        // Driver controls
        m_Driver.cross.whenPressed(new ToggleChargeur(chargeurSubsystem));
        m_Driver.triangle.whenPressed(new ToggleTrappe(trieurSubsystem));
        m_Driver.square.whenPressed(new ShootHighSpeedRevolutionTeleOp(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));
        m_Driver.circle.toggleWhenPressed(new ModeRamassageTeleOp(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));

        m_Driver.start.whenPressed(new SwitchTeamAndFlipPose(drivePedroSubsystem, hubsSubsystem));
        m_Driver.back.whenPressed(new ResetHeadingFCDrive(drivePedroSubsystem));

        m_Driver.bump_left.toggleWhenPressed(
                new SlowDrive(drivePedroSubsystem),
                new MaxPowerDrive(drivePedroSubsystem));

        m_Driver.bump_right.whenPressed(new SwitchAimLockType(drivePedroSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));

        // Operator controls
        m_Operator.dpad_up.whenPressed(new MoulinHighSpeedRevolution(trieurSubsystem, shooterSubsystem));
        m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
        m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));

        m_Operator.back.whenPressed(new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(!trieurSubsystem.wantsMotifShoot())));
        m_Operator.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
        m_Operator.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

        m_Operator.right_stick_button.whenPressed(new SwitchUsageStateShooter(shooterSubsystem, drivePedroSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));

        m_Operator.square.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, LONG_SHOOT_SHOOTER_VELOCITY),
                new StopShooter(shooterSubsystem), true);
        m_Operator.cross.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, MID_SHOOT_SHOOTER_VELOCITY),
                new StopShooter(shooterSubsystem), true);
        m_Operator.triangle.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, CLOSE_SHOOT_SHOOTER_VELOCITY),
                new StopShooter(shooterSubsystem), true);
        m_Operator.circle.whenPressed(new ParallelCommandGroup(
                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true))));

        new Trigger(() -> m_Operator.getRightTriggerValue() > 0.2)
                .whenActive(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
        new Trigger(() -> m_Operator.getLeftTriggerValue() > 0.2)
                .whenActive(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));

    }

}
