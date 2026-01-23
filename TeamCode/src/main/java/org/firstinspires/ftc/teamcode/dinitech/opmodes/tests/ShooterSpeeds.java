package org.firstinspires.ftc.teamcode.dinitech.opmodes.tests;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleSlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.ToggleMaxShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

@TeleOp(name = "LanceurSpeeds - Dinitech", group = "Test")
public class ShooterSpeeds extends DinitechRobotBase {
    private GamepadSubsystem gamepadSubsystem;
    private GamepadWrapper m_Driver;
    private GamepadWrapper m_Operator;
    private TrieurSubsystem trieurSubsystem;
    private VisionSubsystem visionSubsystem;

    private ShooterSubsystem shooterSubsystem;
    private ChargeurSubsystem chargeurSubsystem;
    private DrivePedroSubsystem drivePedroSubsystem;

        /**
         * Initialize the teleop OpMode, gamepads, buttons, and default commands.
         */
        @Override
        public void initialize() {
                super.initialize();

                gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
                register(gamepadSubsystem);
                m_Driver = gamepadSubsystem.driver;
                m_Operator = gamepadSubsystem.operator;

                visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
                register(visionSubsystem);

                drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, BEGIN_POSE,
                                telemetryM);
                register(drivePedroSubsystem);

                trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
                register(trieurSubsystem);

                chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
                register(chargeurSubsystem);

                shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
                register(shooterSubsystem);

                setupGamePadsButtonBindings();
//                customSetupGamePadsButtonBindings();

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
//            visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));
            drivePedroSubsystem.setDefaultCommand(new RobotCentricDrive(drivePedroSubsystem, gamepadSubsystem));
            shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));
//                shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));


            //Overwrite m1 & m2
            m_Driver.m1Button.whenPressed(new InstantCommand());
            m_Driver.m2Button.whenPressed(new InstantCommand());
            m_Operator.m1Button.whenPressed(new InstantCommand());
            m_Operator.m2Button.whenPressed(new InstantCommand());

            // Full stop robot
            m_Driver.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));
            m_Operator.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));

            // Driver controls
            m_Driver.circle.whenPressed(new ToggleMaxShooter(shooterSubsystem));
            m_Driver.cross.whenPressed(new ToggleChargeur(chargeurSubsystem));
            m_Driver.triangle.whenPressed(new ToggleTrappe(trieurSubsystem, gamepadSubsystem));
//            m_Driver.square.whenPressed(new ShootRevolution(trieurSubsystem, shooterSubsystem));

            m_Driver.bump_left.whenPressed(new ToggleSlowDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));
            m_Driver.bump_right
                            .whenPressed(new ToggleVisionDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));



            // Operator controls
//            m_Operator.dpad_up.toggleWhenPressed(new ShootRevolution(trieurSubsystem, shooterSubsystem));
            m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
            m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));

            m_Operator.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
            m_Operator.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

//            m_Operator.right_stick_button.whenPressed(new ToggleVisionShooter(shooterSubsystem, visionSubsystem, gamepadSubsystem));

//            m_Operator.cross.whenPressed(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
//            m_Operator.square.whenPressed(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
//            m_Operator.triangle.whenPressed(new ArtefactPickAway(trieurSubsystem, gamepadSubsystem));
            m_Operator.circle.toggleWhenPressed(new AutomaticArtefactPickAway(trieurSubsystem, gamepadSubsystem));


//                 Automatic trigger: when trieur becomes full, spin up shooter to max speed and
//                 Moulin Motif Ready
//            new Trigger(trieurSubsystem::getIsFull)
//                    .whenActive(new ParallelCommandGroup(new ReadyMotif(trieurSubsystem, visionSubsystem,
//                            gamepadSubsystem),
//                            new VisionShooter(shooterSubsystem, visionSubsystem, false)
//                    ));

                // Example trigger usage (you can uncomment and add commands as needed)
        }

        private void customSetupGamePadsButtonBindings(){
            gamepadSubsystem.setDefaultCommand(new DefaultGamepadCommand(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
//            visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));
            drivePedroSubsystem.setDefaultCommand(new RobotCentricDrive(drivePedroSubsystem, gamepadSubsystem));
//            shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));
//                shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));

            //Overwrite m1 & m2
            m_Driver.m1Button.whenPressed(new InstantCommand());
            m_Driver.m2Button.whenPressed(new InstantCommand());
            m_Operator.m1Button.whenPressed(new InstantCommand());
            m_Operator.m2Button.whenPressed(new InstantCommand());

            // Full stop robot
            m_Driver.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));
            m_Operator.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, shooterSubsystem, chargeurSubsystem));

            // Driver controls
            m_Driver.circle.whenPressed(new ToggleMaxShooter(shooterSubsystem));
//            m_Driver.cross.whenPressed(new ToggleChargeur(chargeurSubsystem));
            m_Driver.triangle.whenPressed(new ToggleTrappe(trieurSubsystem, gamepadSubsystem));


//            m_Driver.bump_left.whenPressed(new ToggleSlowDrive(drivePedroSubsystem, gamepadSubsystem));
            m_Driver.bump_right
                    .whenPressed(new ToggleVisionDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem));

            // Operator controls
//            m_Operator.dpad_up.toggleWhenPressed(new ShootRevolution(trieurSubsystem, shooterSubsystem));
            m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
            m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));

            m_Operator.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
            m_Operator.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

//            m_Operator.right_stick_button.whenPressed(new ToggleVisionShooter(shooterSubsystem, visionSubsystem, gamepadSubsystem));

//            m_Operator.cross.whenPressed(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
//            m_Operator.square.whenPressed(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
//            m_Operator.triangle.whenPressed(new ArtefactPickAway(trieurSubsystem, gamepadSubsystem));
//            m_Operator.circle.toggleWhenPressed(new AutomaticArtefactPickAway(trieurSubsystem,
//                    chargeurSubsystem, gamepadSubsystem));


//                 Automatic trigger: when trieur becomes full, spin up shooter to max speed and
//                 Moulin Motif Ready
//            new Trigger(trieurSubsystem::getIsFull)
//                    .whenActive(new ParallelCommandGroup(new ReadyMotif(trieurSubsystem, visionSubsystem,
//                            gamepadSubsystem),
//                            new VisionShooter(shooterSubsystem, visionSubsystem, false)
//                    ));
        }
}
