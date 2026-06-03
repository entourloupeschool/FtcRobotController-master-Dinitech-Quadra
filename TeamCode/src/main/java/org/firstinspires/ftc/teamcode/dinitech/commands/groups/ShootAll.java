package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.SPEED_MARGIN_SUPER_INTEL;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.END_WAIT_HIGH_SPEED_TRIEUR;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.SCALE_AFTER_HIGH_SPEED_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShootWaitVelocity;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.WaitReadyShootTrappeFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.HashMap;


public class ShootAll extends SelectCommand {
    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, boolean waitInitSpeed, boolean waitEachSpeed, boolean withShooterOvercurrent) {
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem, shooterSubsystem, waitInitSpeed),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            EndShootAll()));

                    put(2, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem, shooterSubsystem, waitInitSpeed),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            new WaitShoot(shooterSubsystem, trieurSubsystem, withShooterOvercurrent),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            EndShootAll()));

                    put(3, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem, shooterSubsystem, waitInitSpeed),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            new WaitShoot(shooterSubsystem, trieurSubsystem, withShooterOvercurrent),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            new WaitShoot(shooterSubsystem, trieurSubsystem, withShooterOvercurrent),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            EndShootAll()));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }
    public static ParallelCommandGroup BeginShootAll(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem, ShooterSubsystem shooterSubsystem, boolean waitInitSpeed){
        return new ParallelCommandGroup(
                new WaitReadyShootTrappeFinger(trieurSubsystem),
                new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                new ConditionalCommand(
                        new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                        new InstantCommand(),
                        ()->waitInitSpeed)
        );
    }

    public static ConditionalCommand ShootAllVeloCondition(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, boolean waitEachSpeed){
        return new ConditionalCommand(
                new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                new MoulinNextArtefactShoot(trieurSubsystem),
                ()->waitEachSpeed);
    }
    
    public static WaitCommand EndShootAll(){
        return new WaitCommand(END_WAIT_HIGH_SPEED_TRIEUR);
    }
}

