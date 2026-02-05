package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_SMALL_TRIANGLE_SHOOT_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_RED_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LONG_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MID_SHOOT_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_SMALL_TRIANGLE_SHOOT_POSE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.DefaultHubsCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetHeadingFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetPoseFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ToggleVisionDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.ToggleUsageStateShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinAntiRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRotate;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OptimizedUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootGreen;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootPurple;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.modes.ModeRamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.Objects;

@TeleOp(name = "GornetixTeleOp - Dinitech", group = "TeleOp")
public class GornetixTeleOp extends GornetixRobotBase {
    public GamepadSubsystem gamepadSubsystem;
    public GamepadWrapper m_Driver;
    public GamepadWrapper m_Operator;
    public TrieurSubsystem trieurSubsystem;
    public VisionSubsystem visionSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public ChargeurSubsystem chargeurSubsystem;
    public DrivePedroSubsystem drivePedroSubsystem;

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
            register(gamepadSubsystem);
            m_Driver = gamepadSubsystem.getDriver();
            m_Operator = gamepadSubsystem.getOperator();

            visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
            register(visionSubsystem);

            drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, Objects.requireNonNullElseGet(PoseStorage.getLastPose(), () -> BEGIN_POSE), telemetryM);
            register(drivePedroSubsystem);
            PoseStorage.clearLastPose();

            trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
            register(trieurSubsystem);

            chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
            register(chargeurSubsystem);

            shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
            register(shooterSubsystem);

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
        hubsSubsystem.setDefaultCommand(new DefaultHubsCommand(hubsSubsystem, trieurSubsystem, this::getOnBlueTeam));
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
        m_Driver.square.whenPressed(new ShootRevolution(trieurSubsystem));

        m_Driver.bump_left.toggleWhenPressed(
                new InstantCommand(
                        () -> drivePedroSubsystem.setDefaultCommand(new SlowDrive(drivePedroSubsystem, gamepadSubsystem))),
                new InstantCommand(
                        () -> drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem))));
        m_Driver.bump_right.whenPressed(new ToggleVisionDrive(drivePedroSubsystem, visionSubsystem, gamepadSubsystem, this::getOnBlueTeam));

        m_Driver.left_stick_button.whenPressed(new ResetHeadingFCDrive(drivePedroSubsystem));

        m_Driver.dpad_down.whenPressed(new ResetPoseFCDrive(drivePedroSubsystem, getOnBlueTeam() ? CLOSE_SHOOT_BLUE_POSE : CLOSE_SHOOT_RED_POSE));
        m_Driver.dpad_up.whenPressed(new ResetPoseFCDrive(drivePedroSubsystem, getOnBlueTeam() ? BLUE_SMALL_TRIANGLE_SHOOT_POSE : RED_SMALL_TRIANGLE_SHOOT_POSE));

        // Operator controls
        m_Operator.dpad_up.whenPressed(new MoulinRevolution(trieurSubsystem));
        m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
        m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));
        m_Operator.dpad_down.whenPressed(new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem));

        m_Operator.bump_right.whileHeld(new MoulinRotate(trieurSubsystem));
        m_Operator.bump_left.whileHeld(new MoulinAntiRotate(trieurSubsystem));

        m_Operator.right_stick_button.whenPressed(new ToggleUsageStateShooter(shooterSubsystem, drivePedroSubsystem, visionSubsystem, gamepadSubsystem, this::getGoalPose));

        m_Operator.square.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, LONG_SHOOT_SHOOTER_VELOCITY),
                new SetVelocityShooter(shooterSubsystem, 0), true);
        m_Operator.cross.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, MID_SHOOT_SHOOTER_VELOCITY),
                new SetVelocityShooter(shooterSubsystem, 0), true);
        m_Operator.triangle.toggleWhenPressed(new SetVelocityShooter(shooterSubsystem, CLOSE_SHOOT_SHOOTER_VELOCITY),
                new SetVelocityShooter(shooterSubsystem, 0), true);

        m_Operator.circle.toggleWhenPressed(new ModeRamassageAuto(trieurSubsystem, chargeurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT));

        m_Operator.start.whenPressed(new InstantCommand(() -> {
            setOnBlueTeam(!getOnBlueTeam());
        }));

        new Trigger(() -> m_Operator.getRightTriggerValue() > 0.2)
                .whenActive(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
        new Trigger(() -> m_Operator.getLeftTriggerValue() > 0.2)
                .whenActive(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));

        new Trigger(trieurSubsystem::getIsFull)
                .whenActive(new StopChargeur(chargeurSubsystem));
    }

}
