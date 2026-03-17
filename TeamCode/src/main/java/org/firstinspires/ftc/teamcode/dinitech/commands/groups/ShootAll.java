package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_SUPER_INTEL;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShootWaitVelocity;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextArtefactShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.HashMap;


public class ShootAll extends SelectCommand {
    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        super(
            new HashMap<Object, Command>(){{
                put(0, new InstantCommand());

                put(1, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));

                put(2, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));

                put(3, new SequentialCommandGroup(
                        new OpenWaitTrappe(trieurSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitShoot(shooterSubsystem),
                        new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
                        new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));}},

            trieurSubsystem::getHowManyArtefacts
        );
    }

    public ShootAll(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, boolean waitSpeed) {
        super(
                new HashMap<Object, Command>(){{
                    put(0, new InstantCommand());

                    put(1, new SequentialCommandGroup(
                            new OpenWaitTrappe(trieurSubsystem),
                            new ConditionalCommand(
                                    new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                    new InstantCommand(),
                                    ()->waitSpeed),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));

                    put(2, new SequentialCommandGroup(
                            new OpenWaitTrappe(trieurSubsystem),
                            new ConditionalCommand(
                                    new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                    new InstantCommand(),
                                    ()->waitSpeed),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));

                    put(3, new SequentialCommandGroup(
                            new OpenWaitTrappe(trieurSubsystem),
                            new ConditionalCommand(
                                    new WaitUntilCommand(()->shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL)),
                                    new InstantCommand(),
                                    ()->waitSpeed),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR),
                            new MoulinNextArtefactShoot(trieurSubsystem),
                            new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)));}},

                trieurSubsystem::getHowManyArtefacts
        );
    }
}
