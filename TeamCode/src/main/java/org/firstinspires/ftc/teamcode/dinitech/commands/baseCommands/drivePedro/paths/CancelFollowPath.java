package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;

/**
 * Command to cancel a path.
 * </pre>
 */
public class CancelFollowPath extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;

    /**
     * Creates a CancelFollowPath command.
     * If there is a current path, it is canceled.
     *
     * @param drivePedroSubsystem The drive subsystem
     */
    public CancelFollowPath(DrivePedroSubsystem drivePedroSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;

        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize() {
        if (drivePedroSubsystem.getDriveUsage() == DrivePedroSubsystem.DriveUsage.AUTO){
            drivePedroSubsystem.getDrive().startTeleOpDrive(true);
        }
        drivePedroSubsystem.setDriveReference(DrivePedroSubsystem.DriveReference.FC);
        drivePedroSubsystem.setDriveUsage(DrivePedroSubsystem.DriveUsage.TELE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
