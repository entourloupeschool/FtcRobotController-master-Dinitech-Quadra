package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.FOLLOWER_T_POSITION_END_TELEOP;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.FIELD_CENTER_90HEADING_POSE;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.SHOOT_REVOLUTION_THEN_WAIT;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.MOULIN_ROTATE_SPEED_CONTINUOUS;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.StopRobot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.ToggleChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.MaxPowerDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.PedroAimLockedDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetHeadingFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.ResetPoseFCDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SlowDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.SwitchAimLockType;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.gamepad.DefaultGamepadCommand;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.PedroShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.WaitVelocityShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SwitchUsageStateShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinHighSpeedIntel;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinHighSpeedRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNext;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinRevolution;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.PrepShootTrieur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.ToggleTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.OnlyMotifDetection;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAllAnyWay;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootGreen;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootPurple;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.GornetixGamepads;
import org.firstinspires.ftc.teamcode.dinitech.other.MotifStorage;
import org.firstinspires.ftc.teamcode.dinitech.other.MoulinPositionColorsStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class TeleOpBase extends GornetixGamepads {

    private int lastHowManyArtefacts = 0;
    private int currentGetHowManyArtefacts = 0;

    @Override
    public void initialize() {
            super.initialize();

            drivePedroSubsystem.setPose(FIELD_CENTER_90HEADING_POSE);
            drivePedroSubsystem.setFollowerTEnd(FOLLOWER_T_POSITION_END_TELEOP);
            drivePedroSubsystem.startTeleOpDrive(true);

            if (MoulinPositionColorsStorage.getLastMoulinPositionColors() != null){
                TrieurSubsystem.ArtifactColor[] newMPC = MoulinPositionColorsStorage.getLastMoulinPositionColors();
                for (int i = 0; i < newMPC.length; i++){
                    TrieurSubsystem.ArtifactColor color = newMPC[i];
                    if (color != TrieurSubsystem.ArtifactColor.NONE && color != TrieurSubsystem.ArtifactColor.IMP){
                        trieurSubsystem.setMoulinStoragePositionColor(i+1, color);
                    }
                }
                MoulinPositionColorsStorage.clearLastMoulinPositionColors();

                trieurSubsystem.setHowManyArtefacts(MoulinPositionColorsStorage.getHowManyArtefactStorage());
                lastHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();
                MoulinPositionColorsStorage.clearHowManyArtefactStorage();
            }

            if (MotifStorage.getMotifNumber() != -1){
                visionSubsystem.setCachedMotif(MotifStorage.getMotifNumber());
                MotifStorage.clearMotifNumber();
            } else {visionSubsystem.setDefaultCommand(new OnlyMotifDetection(visionSubsystem));}

            gamepadSubsystem.setDefaultCommand(new DefaultGamepadCommand(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, gamepadSubsystem));
            drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem));
            shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem));

            setupGamePadsButtonBindings();

            new MoulinCalibrationSequence(trieurSubsystem).schedule();
    }

    @Override
    public void run() {
        currentGetHowManyArtefacts = trieurSubsystem.getHowManyArtefacts();
        if (lastHowManyArtefacts != currentGetHowManyArtefacts){
            lastHowManyArtefacts = currentGetHowManyArtefacts;
            if (currentGetHowManyArtefacts == 0) modeRamassage();

            if (currentGetHowManyArtefacts == 2) new InstantCommand(()->shooterSubsystem.setDefaultCommand(new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem)), shooterSubsystem).schedule();

            if(currentGetHowManyArtefacts == 3) modeShoot();
        }

        double rightTriggerValue = m_Operator.getRightTriggerValue();
        double leftTriggerValue = m_Operator.getLeftTriggerValue();
        if (rightTriggerValue > 0.05)trieurSubsystem.incrementMoulinTargetPosition(rightTriggerValue * rightTriggerValue * MOULIN_ROTATE_SPEED_CONTINUOUS);
        if (leftTriggerValue > 0.05)trieurSubsystem.incrementMoulinTargetPosition(-leftTriggerValue * leftTriggerValue * MOULIN_ROTATE_SPEED_CONTINUOUS);

        super.run();
    }

    private void setupGamePadsButtonBindings() {
        // Full stop robot
        m_Driver.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem));
        m_Operator.touchpadButton.whenActive(new StopRobot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem));

        // Driver controls
        m_Driver.cross.whenPressed(new ToggleChargeur(chargeurSubsystem));

        m_Driver.triangle.whenPressed(new ToggleTrappe(trieurSubsystem));
        m_Driver.square.whenPressed(new ShootAllAnyWay(trieurSubsystem, shooterSubsystem, chargeurSubsystem));

        m_Driver.circle.toggleWhenPressed(new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem, false));

        m_Driver.start.whenPressed(new ResetPoseFCDrive(drivePedroSubsystem, hubsSubsystem));
        m_Driver.back.whenPressed(new ResetHeadingFCDrive(drivePedroSubsystem));

        m_Driver.bump_left.toggleWhenPressed(
                new SlowDrive(drivePedroSubsystem),
                new MaxPowerDrive(drivePedroSubsystem));

        m_Driver.bump_right.whenPressed(new SwitchAimLockType(drivePedroSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));

        // Operator controls
//        m_Operator.dpad_up.whenPressed(new MoulinHighSpeedIntel(trieurSubsystem, shooterSubsystem));
        m_Operator.dpad_up.whenPressed(new MoulinRevolution(trieurSubsystem));

        m_Operator.dpad_right.whenPressed(new MoulinNextNext(trieurSubsystem));
        m_Operator.dpad_left.whenPressed(new MoulinNext(trieurSubsystem));


        m_Operator.dpad_down.toggleWhenPressed(
                new InverseMaxPowerChargeur(chargeurSubsystem),
                new StopChargeur(chargeurSubsystem), true);

        m_Operator.back.whenPressed(new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(!trieurSubsystem.wantsMotifShoot())));

        m_Operator.start.whenPressed(new InstantCommand(()->{
            trieurSubsystem.setHowManyArtefacts(0);
            trieurSubsystem.clearAllStoredColors();}));

        m_Operator.right_stick_button.whenPressed(new SwitchUsageStateShooter(shooterSubsystem, drivePedroSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem));

        m_Operator.square.toggleWhenPressed(new WaitVelocityShooter(shooterSubsystem, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),
                new StopShooter(shooterSubsystem), true);
        m_Operator.cross.toggleWhenPressed(new WaitVelocityShooter(shooterSubsystem, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY * 0.8),
                new StopShooter(shooterSubsystem), true);
        m_Operator.triangle.toggleWhenPressed(new WaitVelocityShooter(shooterSubsystem, AUDIENCE_SHOOT_AUTO_SHOOTER_VELOCITY),
                new StopShooter(shooterSubsystem), true);

        m_Operator.circle.whenPressed(new SequentialCommandGroup(
                new InstantCommand(()->trieurSubsystem.setWantsMotifShoot(true)),
                new PrepShootTrieur(trieurSubsystem, visionSubsystem, gamepadSubsystem)));

        m_Operator.bump_right.whenPressed(new ShootPurple(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
        m_Operator.bump_left.whenPressed(new ShootGreen(trieurSubsystem, shooterSubsystem, gamepadSubsystem));
    }

    protected void modeShoot(){
        new InstantCommand(()->shooterSubsystem.setDefaultCommand(new PedroShooter(shooterSubsystem, drivePedroSubsystem, hubsSubsystem)), shooterSubsystem).schedule();
        new InstantCommand(()->drivePedroSubsystem.setDefaultCommand(new PedroAimLockedDrive(drivePedroSubsystem, gamepadSubsystem, hubsSubsystem)), drivePedroSubsystem).schedule();
        new StopChargeur(chargeurSubsystem).schedule();
    }

    private void modeRamassage() {
        new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem)), shooterSubsystem).schedule();
        new InstantCommand(()->drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem)), drivePedroSubsystem).schedule();
        new SequentialCommandGroup(
                new WaitCommand(SHOOT_REVOLUTION_THEN_WAIT),
                new MoulinCalibrationSequence(trieurSubsystem),
                new WaitCommand(100),
                new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem, false)).schedule();
    }

}
