package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.velocityTrieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.VelocityTrieurSubsystem;

/**
 * A command for rotating the moulin to a specific position, designed for the {@link VelocityTrieurSubsystem}.
 * <p>
 * This command is the velocity-controlled counterpart to the original {@link org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinToPosition}.
 * It performs the same function—rotating the moulin to a target position—but is specifically
 * built to operate with the {@code VelocityTrieurSubsystem}.
 */
public class MoulinToPosition extends CommandBase {
    protected final VelocityTrieurSubsystem trieurSubsystem;
    protected int moulinTargetPosition;
    protected boolean makeShort;

    /**
     * Creates a new MoulinToPosition command for the velocity-controlled sorter.
     *
     * @param trieurSubsystem      The {@link VelocityTrieurSubsystem} to control.
     * @param moulinTargetPosition The target logical position (1-6) for the moulin.
     * @param makeShort            If true, the moulin will take the shortest path; otherwise, it will rotate forward.
     */
    public MoulinToPosition(VelocityTrieurSubsystem trieurSubsystem, int moulinTargetPosition, boolean makeShort) {
        this.trieurSubsystem = trieurSubsystem;
        this.moulinTargetPosition = moulinTargetPosition;
        this.makeShort = makeShort;

        addRequirements(trieurSubsystem);
    }

    /**
     * Initiates the rotation by commanding the subsystem to move to the target position.
     */
    @Override
    public void initialize() {
        trieurSubsystem.rotateToMoulinPosition(moulinTargetPosition, makeShort);
        trieurSubsystem.setMoulinPower(POWER_MOULIN_ROTATION);
    }

    /**
     * The command is finished when the subsystem indicates that motor power should be cut.
     *
     * @return True if the command should end.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.shouldMoulinStopPower();
    }

    /**
     * Stops the motor at the end of the command and handles interruptions.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        trieurSubsystem.setMoulinPower(0);

        if (interrupted) {
            trieurSubsystem.resetTargetMoulinMotor();
        }
    }
}
