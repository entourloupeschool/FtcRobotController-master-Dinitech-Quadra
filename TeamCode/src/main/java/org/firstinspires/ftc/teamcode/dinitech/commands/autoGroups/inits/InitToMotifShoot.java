package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.FollowPath;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class InitToMotifShoot extends SequentialCommandGroup {

    public InitToMotifShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, ChargeurSubsystem chargeurSubsystem, Pose shootPose, double shootVelocity, double endTime){
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(),
                                new SetVelocityShooterRequire(shooterSubsystem, shootVelocity)),

                        new ParallelCommandGroup(
                                // Go to Shooting Pos
                                OptimalPath.line(drivePedroSubsystem, shootPose, 1, true),
                                new SequentialCommandGroup(
                                        // Race: wait for hasColorOrder OR wait for path to finish
                                        new ParallelRaceGroup(
                                                new WaitUntilCommand(visionSubsystem::hasMotif),
                                                new WaitUntilCommand(drivePedroSubsystem::isPathQuasiDone)),
                                        // Only run ReadyMotif if hasColorOrder became true, otherwise skip
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new ReadyMotif(trieurSubsystem, visionSubsystem),
                                                        new WaitOpenTrappe(trieurSubsystem)),
                                                new WaitOpenTrappe(trieurSubsystem),
                                                visionSubsystem::hasMotif)))),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem)
        );
    }
}
