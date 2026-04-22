package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.SHOOT_REVOLUTION_THEN_WAIT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.ToClosestShootPose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.ToPickPose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.TeleShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinCalibrationSequence;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;

public class TeleOpBaseAutomations extends TeleOpBase {

    @Override
    public void initialize(){
        super.initialize();
        m_Driver.touchpadButton.whenActive(new ToPickPose(drivePedroSubsystem, hubsSubsystem, gamepadSubsystem));
    }


    /**
     * Over ride modeShoot()
     */
    @Override
    public void modeShoot(){
        schedule(new SequentialCommandGroup(
                new StopChargeur(chargeurSubsystem),
                new ToClosestShootPose(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, hubsSubsystem, gamepadSubsystem)));
    }

    /**
     * Overrider modeRamassage()
     */
    @Override
    public void modeRamassage(){
        schedule(
                new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new TeleShooter(shooterSubsystem, gamepadSubsystem)), shooterSubsystem),
                new InstantCommand(()->drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem)), drivePedroSubsystem),
                new ParallelCommandGroup(
                        new ToPickPose(drivePedroSubsystem, hubsSubsystem, gamepadSubsystem),
                        new SequentialCommandGroup(
                                new WaitCommand(SHOOT_REVOLUTION_THEN_WAIT),
                                new RamassageAuto(trieurSubsystem, visionSubsystem, gamepadSubsystem, chargeurSubsystem, false))));

    }
}
