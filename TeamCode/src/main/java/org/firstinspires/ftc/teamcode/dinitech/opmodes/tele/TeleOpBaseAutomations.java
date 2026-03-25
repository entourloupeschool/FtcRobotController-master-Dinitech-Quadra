package org.firstinspires.ftc.teamcode.dinitech.opmodes.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.ToClosestShootPose;

public class TeleOpBaseAutomations extends TeleOpBase {


    /**
     * Over ride modeShoot()
     */
    @Override
    public void modeShoot(){
        schedule(new SequentialCommandGroup(
                new StopChargeur(chargeurSubsystem),
                new ToClosestShootPose(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem, gamepadSubsystem)
                .interruptOn(() ->
                        Math.hypot(m_Driver.getLeftX(), m_Driver.getLeftY()) > 0.02
                                || Math.hypot(m_Driver.getRightX(), m_Driver.getRightY()) > 0.02)));
    }
}
