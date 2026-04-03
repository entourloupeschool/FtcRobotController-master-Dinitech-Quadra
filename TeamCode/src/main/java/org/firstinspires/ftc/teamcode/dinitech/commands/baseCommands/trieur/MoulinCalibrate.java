package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.INTERVALLE_TICKS_MOULIN_DOUBLE;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.MAGNETIC_ON_MOULIN_POSITION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.MOULIN_ROTATE_SPEED_CALIBRATION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.OFFSET_MAGNETIC_POS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

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
    private boolean leftMagneticSwitch;
    private boolean isOnMagneticSwitch;

    /**
     * Creates a new MoulinCalibrate command.
     *
     * @param trieurSubsystem The sorter subsystem to be calibrated.
     */
    public MoulinCalibrate(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    @Override
    public void initialize() {
        leftMagneticSwitch = false;
        isOnMagneticSwitch = false;
    }

    /**
     * Continuously increments the motor's target position to ensure it keeps rotating
     * until the magnetic switch is found.
     */
    @Override
    public void execute() {
        if (!isOnMagneticSwitch) trieurSubsystem.incrementMoulinEncoderTargetPosition(MOULIN_ROTATE_SPEED_CALIBRATION);

        if (trieurSubsystem.isMagneticSwitch() && !isOnMagneticSwitch) {
            isOnMagneticSwitch = true;
//            trieurSubsystem.resetTargetMoulinMotor();
            trieurSubsystem.resetMoulinEncoderTarget();
            trieurSubsystem.incrementMoulinEncoderTargetPosition(INTERVALLE_TICKS_MOULIN_DOUBLE + OFFSET_MAGNETIC_POS);
            trieurSubsystem.setMoulinPosition(Moulin.getNPreviousPosition(MAGNETIC_ON_MOULIN_POSITION, 1));
        }

        if (isOnMagneticSwitch && !trieurSubsystem.isMagneticSwitch()) leftMagneticSwitch = true;
    }

    @Override
    public boolean isFinished() {
        return leftMagneticSwitch;
    }

    @Override
    public void end(boolean interrupted) {
        trieurSubsystem.setHasInitCalibration(true);
    }

}
