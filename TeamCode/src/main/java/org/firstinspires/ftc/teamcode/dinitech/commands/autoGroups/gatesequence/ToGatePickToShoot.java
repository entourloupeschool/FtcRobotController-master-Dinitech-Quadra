package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.GATE_UNSHORTCUT_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.RADIUS_RAMP_PICK;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.T_PARAMETRIC_DONT_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.TILE_DIM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem.INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths.OptimalPath;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.SetVelocityShooterRequire;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.WaitReadyShootTrappeFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TrieurReadyEmptyStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefactOptimized;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class ToGatePickToShoot extends SequentialCommandGroup {
    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, Pose openRampPose, Pose endRampPose, Pose shootPose, double shooterVelocity, boolean shortcutBackPath){
        Pose interRampPose = endRampPose
                .withX(endRampPose.getX() + RADIUS_RAMP_PICK*(endRampPose.getX() > 72 ? -1 : 1))
                .withY((endRampPose.getY() + openRampPose.getY()) / 2);


        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.curve(drivePedroSubsystem,
                                openRampPose.withX(openRampPose.getX() + GATE_UNSHORTCUT_SCALE*TILE_DIM*(openRampPose.getX() > 72 ? -1 : 1)),
                                openRampPose, 1, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 4), trieurSubsystem),
                                new ParallelCommandGroup(
                                        new TrieurReadyEmptyStorage(trieurSubsystem),
                                        new MaxPowerChargeur(chargeurSubsystem)),
                                new TryDetectArtefactOptimized(trieurSubsystem),
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 2), trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),

                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem)),

                        createCurveStayRampPass(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose, openRampPose)),

                new ParallelCommandGroup(
                        new ParallelCommandGroup(
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new ReadyMotif(trieurSubsystem, visionSubsystem),
                                                new InstantCommand(),
                                                ()->trieurSubsystem.wantsMotifShoot()),
                                        new WaitReadyShootTrappeFinger(trieurSubsystem),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InverseMaxPowerChargeur(chargeurSubsystem),
                                                        new WaitCommand(INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO)),
                                                new InstantCommand(),
                                                ()->trieurSubsystem.isFull()),
                                        new StopChargeur(chargeurSubsystem))),
                        // Go to Shooting Pos
                        shortcutBackPath ?
                                OptimalPath.line(drivePedroSubsystem,
                                        shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                                OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose.withX(openRampPose.getX() + GATE_UNSHORTCUT_SCALE*TILE_DIM*(openRampPose.getX() > 72 ? -1 : 1)),
                                        shootPose, 1, true)
                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();})),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false)
        );
    }

    public ToGatePickToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, Pose openRampPose, Pose endRampPose, Pose shootPose, double shooterVelocity, boolean shortcutBackPath){
        Pose interRampPose = endRampPose
                .withX(endRampPose.getX() + RADIUS_RAMP_PICK*(endRampPose.getX() > 72 ? -1 : 1))
                .withY((endRampPose.getY() + openRampPose.getY()) / 2);


        addCommands(
                new ParallelCommandGroup(
                        new TrieurReadyEmptyStorage(trieurSubsystem),
                        OptimalPath.curve(drivePedroSubsystem,
                                openRampPose.withX(openRampPose.getX() + GATE_UNSHORTCUT_SCALE*TILE_DIM*(openRampPose.getX() > 72 ? -1 : 1)),
                                openRampPose, 1, true)),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 4), trieurSubsystem),
                                new ParallelCommandGroup(
                                        new TrieurReadyEmptyStorage(trieurSubsystem),
                                        new MaxPowerChargeur(chargeurSubsystem)),
                                new TryDetectArtefactOptimized(trieurSubsystem),
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 3), trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),
                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT * 2), trieurSubsystem),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new MoulinNextEmptyStorage(trieurSubsystem),
                                                new TryDetectArtefactOptimized(trieurSubsystem)),
                                        new InstantCommand(),
                                        trieurSubsystem::getNewRegister),

                                new InstantCommand(()->trieurSubsystem.setDetectionTimeout(MODE_RAMASSAGE_AUTO_TIMEOUT), trieurSubsystem)),

                        createCurveStayRampPass(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose, openRampPose)),

                new ParallelCommandGroup(
                        // Go to Shooting Pos
                        shortcutBackPath ?
                                OptimalPath.line(drivePedroSubsystem,
                                        shootPose, 1, true).withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}) :
                                OptimalPath.curve(drivePedroSubsystem,
                                        openRampPose.withX(openRampPose.getX() + GATE_UNSHORTCUT_SCALE*TILE_DIM*(openRampPose.getX() > 72 ? -1 : 1)),
                                        shootPose, 1, true)
                                .withParametricCallback(T_PARAMETRIC_DONT_SHOOT,
                                        () -> {if (trieurSubsystem.isEmpty()) this.cancel();}),
                        new ParallelCommandGroup(
                                new SetVelocityShooterRequire(shooterSubsystem, shooterVelocity),
                                new SequentialCommandGroup(
                                        new WaitReadyShootTrappeFinger(trieurSubsystem),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InverseMaxPowerChargeur(chargeurSubsystem),
                                                        new WaitCommand(INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO)),
                                                new InstantCommand(),
                                                ()->trieurSubsystem.isFull()),
                                        new StopChargeur(chargeurSubsystem)))),

                new ShootAll(trieurSubsystem, shooterSubsystem, chargeurSubsystem, true, false, false)
        );
    }

    private static SequentialCommandGroup createCurveStayRampPass(
            DrivePedroSubsystem drivePedroSubsystem,
            TrieurSubsystem trieurSubsystem,
            Pose interRampPose,
            Pose endRampPose,
            Pose openRampPose) {

        return new SequentialCommandGroup(
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, openRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, openRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, openRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, endRampPose),
                createAbortableCurve(drivePedroSubsystem, trieurSubsystem, interRampPose, openRampPose));
    }

    private static OptimalPath createAbortableCurve(
            DrivePedroSubsystem drivePedroSubsystem,
            TrieurSubsystem trieurSubsystem,
            Pose interRampPose,
            Pose targetPose) {
        OptimalPath path = OptimalPath.curve(drivePedroSubsystem, interRampPose, targetPose, GATEPICK_POWER, false);
        path.addTemporalCallbacks(() -> {
            if (trieurSubsystem.isFull() || drivePedroSubsystem.getFollower().isRobotStuck()) {
                path.cancel();
            }
        }, 1, 100, 300, 800, 1500);
        return path;
    }
}
