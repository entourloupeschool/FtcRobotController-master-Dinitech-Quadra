package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.cmToInch;
import static org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses.ROTATED_BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.MIN_RANGE_TO_SHOOT_CM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.SPEED_MARGIN;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinToPositionMargin;
import org.firstinspires.ftc.teamcode.dinitech.other.TeamPoses;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command that rotates the moulin one step forward to the next sequential position.
 * <p>
 * This command extends {@link MoulinToPositionMargin}. Unlike its parent, the target position
 * is not specified at construction. Instead, it is dynamically determined in the
 * {@code initialize()} method by getting the current moulin position and calculating the
 * next one. This ensures the command always moves to the correct next slot relative
 * to the state of the robot when the command is executed.
 * <p>
 * This command always rotates in the positive (forward) direction.
 */
public class MoulinNextArtefactShootWaitPedroVelocity extends MoulinToPositionMargin {
    private final ShooterSubsystem shooterSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final HubsSubsystem hubsSubsystem;
    private boolean hasLaunched;
    private double distance;
    /**
     * Creates a new MoulinNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNextArtefactShootWaitPedroVelocity(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, HubsSubsystem hubsSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, -1);
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.hubsSubsystem = hubsSubsystem;
        this.hasLaunched = false;
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        hasLaunched = false;

        Pose basketPose;
        if (hubsSubsystem.getTeam() == TeamPoses.Team.BLUE && drivePedroSubsystem.getDriveUsage() == DrivePedroSubsystem.DriveUsage.TELE){
            basketPose = ROTATED_BLUE_BASKET_POSE;
        } else {
            basketPose = hubsSubsystem.getTeam().getBasketPose();
        }

        distance = drivePedroSubsystem.getPose().distanceFrom(basketPose);

        int currentPos = trieurSubsystem.getMoulinPosition();

        for (int i = 1; i < Moulin.TOTAL_POSITIONS + 1; i++) {
            int previousI = Moulin.getNPreviousPosition(currentPos, i);
            TrieurSubsystem.ArtifactColor color = trieurSubsystem.getMoulinStoragePositionColor(previousI);

            if (color == TrieurSubsystem.ArtifactColor.GREEN || color == TrieurSubsystem.ArtifactColor.PURPLE) {
                super.moulinTargetPosition = Moulin.getOppositePosition(previousI);
                break;
            }
        }

        if (super.moulinTargetPosition == -1){
            super.initialize();
            hasLaunched = true;
        }
    }

    @Override
    public void execute(){
        if (!hasLaunched){
            if (distance < 101){
                super.initialize();
                hasLaunched = true;
            } else if (shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN)) {
                super.initialize();
                hasLaunched = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return hasLaunched && super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (super.moulinTargetPosition != -1) trieurSubsystem.clearMoulinStoragePositionColor(Moulin.getOppositePosition(super.moulinTargetPosition));
        super.end(interrupted);

    }
}
