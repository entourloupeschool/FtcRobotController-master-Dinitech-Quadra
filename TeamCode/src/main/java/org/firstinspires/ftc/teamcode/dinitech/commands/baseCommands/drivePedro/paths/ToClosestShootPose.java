package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getClosestVec2InLaunchZone;

import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.linearSpeedFromPedroRange;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechFollower.AUTO_ROBOT_CONSTRAINTS;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.PedroAimLockedDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAll;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAllAnyWay;
import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

import java.util.function.BooleanSupplier;

/**
 * Path builder tuned to maximize tangent heading travel while minimizing rotation demand.
 *
 * <p>Rules:
 * <ol>
 *   <li>If range is short, use pure linear heading interpolation.</li>
 *   <li>If range is long enough, split heading into:
 *     rotate to tangent -> tangent/reverse tangent -> rotate to final heading.</li>
 *   <li>Pick tangent or reverse-tangent based on the smallest total heading change.</li>
 * </ol>
 */
public class ToClosestShootPose extends OptimalPath {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final TrieurSubsystem trieurSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HubsSubsystem hubsSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    public ToClosestShootPose(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                drivePedroSubsystem,
                OptimalPath.createLineSupplier(
                        drivePedroSubsystem,
                        (currentPose, currentHeading) -> computeShootPose(currentPose, hubsSubsystem, shooterSubsystem)),
                AUTO_ROBOT_CONSTRAINTS, true);

        this.drivePedroSubsystem = drivePedroSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hubsSubsystem = hubsSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void end(boolean interrupted){

        if (interrupted) drivePedroSubsystem.pausePathFollowing();
        else new ShootAllAnyWay(trieurSubsystem, shooterSubsystem).schedule();

        drivePedroSubsystem.startTeleOpDrive(true);

        drivePedroSubsystem.setDefaultCommand(new PedroAimLockedDrive(drivePedroSubsystem, gamepadSubsystem, hubsSubsystem));

        super.end(interrupted);
    }

    @Override
    public Command interruptOn(BooleanSupplier condition) {
        BooleanSupplier reunion = ()->
                Math.hypot(gamepadSubsystem.getDriver().getLeftX(), gamepadSubsystem.getDriver().getLeftY()) > 0.02
                || Math.hypot(gamepadSubsystem.getDriver().getRightX(), gamepadSubsystem.getDriver().getRightY()) > 0.02
                || condition.getAsBoolean();
        return super.interruptOn(reunion);
    }

    private static Pose computeShootPose(Pose currentPose, HubsSubsystem hubsSubsystem, ShooterSubsystem shooterSubsystem) {
        Pose basketPose = hubsSubsystem.getTeam().getBasketPose();
        if (basketPose == null) {
            return currentPose;
        }
        Globals.Vec2 basketVec = new Globals.Vec2(basketPose.getX(), basketPose.getY());


        Pose workingPose = currentPose;
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) workingPose = currentPose.rotate(BLUE_TEAM_HEADING, false);

        Globals.Vec2 currentVec = new Globals.Vec2(workingPose.getX(), workingPose.getY());
        Globals.Vec2 shootVec = getClosestVec2InLaunchZone(currentVec, basketVec,1);

        Globals.Vec2 shootToBasketVec = basketVec.subtract(shootVec);
        double shootToBasketAngle = Math.atan2(shootToBasketVec.y, shootToBasketVec.x);

        Pose shootPose = new Pose(shootVec.x, shootVec.y, shootToBasketAngle);

        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) shootPose = shootPose.rotate(BLUE_TEAM_HEADING, true);

        shooterSubsystem.setVelocity(linearSpeedFromPedroRange(shootVec.euclidianDistance(basketVec)));

        return shootPose;
    }
}