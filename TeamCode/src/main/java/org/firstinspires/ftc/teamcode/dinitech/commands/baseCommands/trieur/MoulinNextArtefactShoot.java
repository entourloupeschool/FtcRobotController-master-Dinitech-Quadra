package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_POSITION_VERY_LOOSE_TOLERANCE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

public class MoulinNextShootIntelSuper extends MoulinToPositionMargin {
    public MoulinNextShootIntelSuper(TrieurSubsystem trieurSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, -1, false, MOULIN_POSITION_VERY_LOOSE_TOLERANCE);
    }

    @Override
    public void initialize() {
        int currentMoulinPos = trieurSubsystem.getMoulinPosition();

        for (int i = 1; i < Moulin.TOTAL_POSITIONS + 1; i++) {
            int previousI = Moulin.getNPreviousPosition(currentMoulinPos, i);
            TrieurSubsystem.ArtifactColor color = trieurSubsystem.getMoulinStoragePositionColor(previousI);

            if (color == TrieurSubsystem.ArtifactColor.GREEN || color == TrieurSubsystem.ArtifactColor.PURPLE) {
                super.moulinTargetPosition = Moulin.getOppositePosition(previousI);
                break;
            }
        }

        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        if (super.moulinTargetPosition != -1) trieurSubsystem.clearMoulinStoragePositionColor(Moulin.getOppositePosition(super.moulinTargetPosition));
        super.end(interrupted);

    }
}
