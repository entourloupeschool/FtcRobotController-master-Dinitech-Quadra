package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.fullsequence;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW_SUPER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW_SUPER_23RD;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.gatesequence.ToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.rowsequence.chained.ToRowToShootChained;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class TwoGateFromGoal extends SequentialCommandGroup {


    public TwoGateFromGoal(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, ChargeurSubsystem chargeurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem, HubsSubsystem hubsSubsystem) {

        addCommands(
                new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, hubsSubsystem),

                //SECOND ROW FIRST
                new ToRowToShootChained(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                        SECOND_ROW_BLUE_POSE, LENGTH_X_ROW_SUPER_23RD, 0.6),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem, GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                new ToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                        hubsSubsystem, GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                //FIRST ROW LAST
                new ToRowToShootChained(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, hubsSubsystem,
                        FIRST_ROW_BLUE_POSE, LENGTH_X_ROW_SUPER, 0.6),

                new ParallelCommandGroup(
                        new StopChargeur(chargeurSubsystem),
                        new StopShooter(shooterSubsystem))

        );

    }



}
