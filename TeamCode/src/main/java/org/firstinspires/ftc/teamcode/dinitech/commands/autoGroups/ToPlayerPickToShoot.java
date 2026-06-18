package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.PLAYER_PICK_RAMASSAGE_LENGTH;


import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.RamassageAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class ToPlayerPickToShoot extends SequentialCommandGroup {

    public ToPlayerPickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, Pose playerPickPose, Pose shootPose, double shootVelocity){
        Pose playerPickEnd1Pose = playerPickPose
                .withX(playerPickPose.getX() + PLAYER_PICK_RAMASSAGE_LENGTH*(playerPickPose.getX() > 72 ? 1 : -1))
                .withHeading(Math.PI);
        Pose playerPickEnd2Pose = playerPickEnd1Pose
                .withY(playerPickEnd1Pose.getY() + 5)
                .withX(playerPickEnd1Pose.getX() + PLAYER_PICK_RAMASSAGE_LENGTH/2*(playerPickPose.getX() > 72 ? 1 : -1))
                .withHeading(0);


        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.line(drivePedroSubsystem,
                                playerPickPose, 1, true)),

                new ParallelCommandGroup(
                        new RamassageAuto(trieurSubsystem, chargeurSubsystem, false),
                        createCurvePlayerPickPass(drivePedroSubsystem, trieurSubsystem, playerPickPose, playerPickEnd1Pose, playerPickEnd2Pose)),

                new SetVelocityShooterRequire(shooterSubsystem, shootVelocity),
                OptimalPath.line(drivePedroSubsystem,
                        shootPose, 1, true),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem,true, true, false)
        );
    }

    private static SequentialCommandGroup createCurvePlayerPickPass(
            DrivePedroSubsystem drivePedroSubsystem,
            TrieurSubsystem trieurSubsystem,
            Pose interPlayerPickPose,
            Pose endPlayerPickPose,
            Pose playerPickPose) {

        return new SequentialCommandGroup(
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, endPlayerPickPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, playerPickPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, endPlayerPickPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, playerPickPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, endPlayerPickPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interPlayerPickPose, playerPickPose));
    }

    private static OptimalPath createAbortableCurve(
            DrivePedroSubsystem drivePedroSubsystem,
            TrieurSubsystem trieurSubsystem,
            Pose interPlayerPickPose,
            Pose targetPose) {
        OptimalPath path = OptimalPath.curve(drivePedroSubsystem, interPlayerPickPose, targetPose, GATEPICK_POWER, false);
        path.addTemporalCallbacks(() -> {
            if (trieurSubsystem.isFull() || drivePedroSubsystem.getFollower().isRobotStuck()) {
                path.cancel();
            }
        }, 1, 100, 300, 800, 1500);
        return path;
    }
}
