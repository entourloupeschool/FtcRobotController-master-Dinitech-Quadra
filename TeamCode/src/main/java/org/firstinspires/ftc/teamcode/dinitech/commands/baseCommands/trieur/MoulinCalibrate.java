package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CALIBRATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OFFSET_MAGNETIC_POS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_CALIBRATION_ROTATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command to calibrate the moulin's position using a magnetic switch.
 * <p>
 * This command slowly rotates the moulin until the magnetic switch is activated. The switch
 * serves as a known physical reference point (a "home" position). Once the switch is
 * triggered, the command finishes, and the moulin's motor encoder is reset in the subsystem,
 * ensuring that the software's logical positions (1-6) accurately correspond to the
 * physical slots of the moulin.
 * <p>
 * This calibration is crucial for maintaining accuracy throughout a match, as encoder values
 * can drift over time.
 */
public class MoulinCalibrate extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new MoulinCalibrate command.
     *
     * @param trieurSubsystem The sorter subsystem to be calibrated.
     */
    public MoulinCalibrate(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Continuously increments the motor's target position to ensure it keeps rotating
     * until the magnetic switch is found.
     */
    @Override
    public void execute() {
        trieurSubsystem.incrementMoulinTargetPosition(MOULIN_ROTATE_SPEED_CALIBRATION);

        if (trieurSubsystem.isMagneticSwitch()) {
            trieurSubsystem.resetTargetMoulinMotor();
            trieurSubsystem.incrementMoulinTargetPosition(INTERVALLE_TICKS_MOULIN + OFFSET_MAGNETIC_POS);
            trieurSubsystem.hardSetMoulinPosition(1);
        }
    }

    /**
     * The command is finished when the magnetic switch is pressed
     *
     * @return True if the command should end.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.isMagneticSwitch();
    }
}
