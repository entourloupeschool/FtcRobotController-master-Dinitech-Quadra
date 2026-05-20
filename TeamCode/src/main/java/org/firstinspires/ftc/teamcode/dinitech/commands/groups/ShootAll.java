package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.SPEED_MARGIN_SUPER_INTEL;
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
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.HashMap;


public class ShootAll extends SelectCommand {
    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem) {
        super(
            new HashMap<Object, Command>(){{
                put(0, new InstantCommand());

                put(1, new SequentialCommandGroup(
                        new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                put(2, new SequentialCommandGroup(
                        new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem, trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                put(3, new SequentialCommandGroup(
                        new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem, trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem, trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));}},

            trieurSubsystem::getHowManyArtefacts
        );
    }

    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, boolean waitSpeed) {
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                                    new ConditionalCommand(
                                            new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                            new InstantCommand(),
                                            ()->waitSpeed)),

                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                    put(2, new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                                    new ConditionalCommand(
                                            new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                            new InstantCommand(),
                                            ()->waitSpeed)),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitShoot(shooterSubsystem, trieurSubsystem),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                    put(3, new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                                    new ConditionalCommand(
                                            new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                            new InstantCommand(),
                                            ()->waitSpeed)),

                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitShoot(shooterSubsystem, trieurSubsystem, false),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitShoot(shooterSubsystem, trieurSubsystem, false),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }

    public ShootAll(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem) {
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                    put(2, new SequentialCommandGroup(

                            new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));

                    put(3, new SequentialCommandGroup(

                            new ArtefactSure(trieurSubsystem, chargeurSubsystem),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand((long) (WAIT_HIGH_SPEED_TRIEUR*SCALE_AFTER_HIGH_SPEED_SHOOT))));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }

}
