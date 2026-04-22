package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.paths;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.SCALER_TO_PICK_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.getPedroPoseFromUnitNormalized;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getClosestVec2InLaunchZone;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.BLUE_TEAM_HEADING;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.linearSpeedFromPedroRange;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro.PedroAimLockedDrive;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootAllAnyWay;
import org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions;
import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

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
public class ToPickPose extends OptimalPath {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    public ToPickPose(DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                drivePedroSubsystem,
                OptimalPath.createLineSupplier(
                        drivePedroSubsystem,
                        (currentPose, currentHeading) -> computePickPose(gamepadSubsystem.getDriver(), hubsSubsystem)),
                1, true);

        this.drivePedroSubsystem = drivePedroSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) drivePedroSubsystem.pausePathFollowing();

        drivePedroSubsystem.startTeleOpDrive(true);
        drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem, gamepadSubsystem));

        super.end(interrupted);
    }

    @Override
    public Command interruptOn(BooleanSupplier condition) {
        BooleanSupplier reunion = ()-> gamepadSubsystem.getDriver().usingSticks() || condition.getAsBoolean();
        return super.interruptOn(reunion);
    }

    private static Pose computePickPose(GamepadWrapper driver, HubsSubsystem hubsSubsystem) {
        Pose pickPose;

        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) pickPose = getPedroPoseFromUnitNormalized(- driver.getTouchpadFinger1Y() * SCALER_TO_PICK_POSE, driver.getTouchpadFinger1X() * SCALER_TO_PICK_POSE, 0);
        else pickPose = getPedroPoseFromUnitNormalized(driver.getTouchpadFinger1Y() * SCALER_TO_PICK_POSE, - driver.getTouchpadFinger1X() * SCALER_TO_PICK_POSE, 0);

        Globals.Vec2 pickPoseVec = FieldDefinitions.middleFieldVec.subtract(new Globals.Vec2(pickPose.getX(), pickPose.getY()));

        pickPose = pickPose.withHeading(Math.atan2(pickPoseVec.y, pickPoseVec.x));

        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE) pickPose = pickPose.rotate(BLUE_TEAM_HEADING, true);

        return pickPose;
    }
}