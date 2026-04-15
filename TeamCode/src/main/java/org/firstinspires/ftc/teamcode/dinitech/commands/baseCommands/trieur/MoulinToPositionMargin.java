package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.MOULIN_POSITION_VERY_LOOSE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.OVER_CURRENT_BACKOFF_TICKS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A fundamental command for rotating the moulin to a specific target position.
 * <p>
 * This command serves as the base for most moulin rotation commands (e.g., {@link MoulinNext},
 * {@link MoulinPrevious}, {@link MoulinRevolution}). It takes a target position and a pathing
 * option, then commands the {@link TrieurSubsystem} to perform the rotation.
 * <p>
 * The command finishes when the moulin either reaches its target or an over-current event
 * is detected. It also includes logic to stop the motor and reset its target if the command
 * is interrupted, preventing unwanted movement.
 */
public class MoulinToPositionMargin extends CommandBase {
    protected final TrieurSubsystem trieurSubsystem;
    protected int moulinTargetPosition;
    protected boolean makeShort;
    protected double margin;
    private boolean inCorrectionOfOvercurrent;

    /**
     * Creates a new MoulinToPosition command.
     *
     * @param trieurSubsystem      The sorter subsystem to control.
     * @param moulinTargetPosition The target logical position (1-6) for the moulin.
     * @param makeShort            If true, the moulin will take the shortest path; otherwise, it will rotate forward.
     * @param margin               Margin to reach to end the command.
     */
    public MoulinToPositionMargin(TrieurSubsystem trieurSubsystem, int moulinTargetPosition, boolean makeShort, double margin) {
        this.trieurSubsystem = trieurSubsystem;
        this.moulinTargetPosition = moulinTargetPosition;
        this.makeShort = makeShort;
        this.margin = margin;

        addRequirements(trieurSubsystem);
    }

    public MoulinToPositionMargin(TrieurSubsystem trieurSubsystem, int moulinTargetPosition){
        this(trieurSubsystem, moulinTargetPosition, false, MOULIN_POSITION_VERY_LOOSE_TOLERANCE);
    }


    /**
     * Initiates the rotation by commanding the subsystem to move to the target position.
     */
    @Override
    public void initialize() {
        if (moulinTargetPosition != -1){
            trieurSubsystem.moulinRotateToPosition(moulinTargetPosition, makeShort);
        }
        inCorrectionOfOvercurrent = false;
    }

    @Override
    public void execute(){
        double beforeOverCurrentTarget = 0;
        double targetCorrection = 0;
        if (trieurSubsystem.getOvercurrentCounts() == 10 && !inCorrectionOfOvercurrent) {
            beforeOverCurrentTarget = trieurSubsystem.getMoulinMotorTargetTicks();
            targetCorrection = trieurSubsystem.getMoulinMotorPosition() + OVER_CURRENT_BACKOFF_TICKS;
            inCorrectionOfOvercurrent = true;
        }
        if (inCorrectionOfOvercurrent) {
            trieurSubsystem.setMoulinMotorTargetPosition(targetCorrection);
            if (trieurSubsystem.isMoulinEncoderCloseToTarget(margin)){
                inCorrectionOfOvercurrent = false;
                trieurSubsystem.setMoulinMotorTargetPosition(beforeOverCurrentTarget);
            }
        }
    }

    /**
     * The command is finished when the subsystem indicates that motor power should be cut.
     *
     * @return True if the command should end.
     */
    @Override
    public boolean isFinished() {
        return (moulinTargetPosition == -1 || trieurSubsystem.isMoulinEncoderCloseToTarget(margin)) && !inCorrectionOfOvercurrent;
    }
}
