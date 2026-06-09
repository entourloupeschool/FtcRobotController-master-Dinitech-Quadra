package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.ROTATED_BLUE_BASKET_POSE;
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
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShootWaitVelocity;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.WaitReadyShootTrappeFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.HashMap;


public class ShootAll extends SelectCommand {
    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, boolean waitInitSpeed, boolean waitEachSpeed, boolean withShooterOvercurrent) {
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitInitSpeed),
                            EndShootAll(trieurSubsystem)));

                    put(2, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitInitSpeed),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            EndShootAll(trieurSubsystem)));

                    put(3, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitInitSpeed),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            ShootAllVeloCondition(trieurSubsystem, shooterSubsystem, waitEachSpeed),
                            EndShootAll(trieurSubsystem)));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }

    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem){
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            EndShootAll(trieurSubsystem)));

                    put(2, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            EndShootAll(trieurSubsystem)));

                    put(3, new SequentialCommandGroup(
                            BeginShootAll(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            WaitShoot.WaitShootFinger(trieurSubsystem),
                            new MoulinNextArtefactShootWaitPedroVelocity(trieurSubsystem, shooterSubsystem, drivePedroSubsystem, hubsSubsystem),
                            EndShootAll(trieurSubsystem)));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }
    public static ParallelCommandGroup BeginShootAll(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem){
        return new ParallelCommandGroup(
                new WaitReadyShootTrappeFinger(trieurSubsystem),
                new ArtefactSure(trieurSubsystem, chargeurSubsystem));
    }

    public static ConditionalCommand ShootAllVeloCondition(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, boolean waitEachSpeed){
        return new ConditionalCommand(
                new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                new MoulinNextArtefactShoot(trieurSubsystem),
                ()->waitEachSpeed);
    }
    
    public static ParallelCommandGroup EndShootAll(TrieurSubsystem trieurSubsystem){
        return new ParallelCommandGroup(
                new WaitCommand(END_WAIT_HIGH_SPEED_TRIEUR),
                new InstantCommand(trieurSubsystem::clearAllStoredColors, trieurSubsystem)
        );
    }
}

