package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleSlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.ToggleVisionTeleStopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.TeleDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootGreen;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootPurple;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.VisionShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassage;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeShoot;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "GornetixTeleOp - Dinitech", group = "TeleOp")
public class GornetixTeleOp extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper m_Driver;
    private GamepadWrapper m_Operator;
        private TrieurSubsystem trieurSubsystem;
        private VisionSubsystem visionSubsystem;

        private ShooterSubsystem shooterSubsystem;
        private ChargeurSubsystem chargeurSubsystem;
        private DriveSubsystem driveSubsystem;

        /**
         * Initialize the teleop OpMode, gamepads, buttons, and default commands.
         */
        @Override
        public void initialize() {
                super.initialize();

                gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetry);
                register(gamepadSubsystem);
                m_Driver = gamepadSubsystem.driver;
                m_Operator = gamepadSubsystem.operator;

                visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
                register(visionSubsystem);

                driveSubsystem = new DriveSubsystem(new DinitechMecanumDrive(hardwareMap, BEGIN_POSE),
                                telemetry);
                register(driveSubsystem);

                trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
                register(trieurSubsystem);

                chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetry);
                register(chargeurSubsystem);

                shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
                register(shooterSubsystem);

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
            gamepadSubsystem.setDefaultCommand(new DefaultGamepadCommand(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
            visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));
            driveSubsystem.setDefaultCommand(new TeleDrive(driveSubsystem, gamepadSubsystem));


            //Overwrite m1 & m2
            m_Driver.m1Button.whenPressed(new InstantCommand());
            m_Driver.m2Button.whenPressed(new InstantCommand());
            m_Operator.m1Button.whenPressed(new InstantCommand());
            m_Operator.m2Button.whenPressed(new InstantCommand());

            // Full stop robot
            m_Driver.touchpadButton.whenActive(new StopRobot(shooterSubsystem, chargeurSubsystem));
            m_Operator.touchpadButton.whenActive(new StopRobot(shooterSubsystem, chargeurSubsystem));

            // Driver controls
//            m_Driver.circle.whenPressed(new ToggleMaxShooter(shooterSubsystem));
            m_Driver.cross.whenPressed(new MaxPowerChargeur(chargeurSubsystem));
            m_Driver.triangle.whenPressed(new ToggleTrappe(trieurSubsystem, gamepadSubsystem));
            m_Driver.square.whenPressed(new ShootRevolution(trieurSubsystem, shooterSubsystem, new VisionShooter(shooterSubsystem, visionSubsystem, false)));

            m_Driver.bump_left.whenPressed(new ToggleSlowDrive(driveSubsystem, gamepadSubsystem));
            m_Driver.bump_right.whenPressed(new ToggleVisionDrive(driveSubsystem, visionSubsystem, gamepadSubsystem));


            // Operator controls
            m_Operator.dpad_up.whenPressed(new MoulinRevolution(trieurSubsystem));
            m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
            m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));

            m_Operator.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
            m_Operator.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

            m_Operator.right_stick_button.whenPressed(new ToggleVisionTeleStopShooter(shooterSubsystem, visionSubsystem, gamepadSubsystem));


            m_Operator.square.whenPressed(new SetVelocityShooter(shooterSubsystem, 1850));
            m_Operator.cross.whenPressed(new SetVelocityShooter(shooterSubsystem, 1600));
            m_Operator.triangle.whenPressed(new SetVelocityShooter(shooterSubsystem, 1400));
            m_Operator.circle.toggleWhenPressed(new ModeRamassage(trieurSubsystem, shooterSubsystem,
                            chargeurSubsystem, gamepadSubsystem));

            new Trigger(() -> m_Operator.getRightTriggerValue() > 0.2)
                    .whenActive(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
            new Trigger(() -> m_Operator.getLeftTriggerValue() > 0.2)
                    .whenActive(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));

            new Trigger(trieurSubsystem::getIsFull)
                    .whenActive(new ModeShoot(driveSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem));

        }

}
