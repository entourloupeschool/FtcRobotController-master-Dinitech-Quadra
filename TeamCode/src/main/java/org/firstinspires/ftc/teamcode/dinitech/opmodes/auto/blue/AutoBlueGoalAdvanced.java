package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.blue;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_END_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_RAMP_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FIRST_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.GATEPICK_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW_SUPER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LENGTH_X_ROW_SUPER_23RD;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LINEAR_HEADING_INTERPOLATION_END_TIME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SECOND_ROW_BLUE_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SUPER_POWER_ROW_PICK_ARTEFACTS;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.InitToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ShootToGatePickToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.ShootToRowToShoot;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.auto.AutoBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;

@Autonomous(name = "BlueGoalAdvanced", group = "Blue")
public class AutoBlueGoalAdvanced extends AutoBase {

    /**
     * Initialize the teleop OpMode, gamepads, buttons, and default commands.
     */
    @Override
    public void initialize() {
            super.initialize();

            hubsSubsystem.setTeam(HubsSubsystem.Team.BLUE);

            drivePedroSubsystem.getDrive().prepAuto(BLUE_GOAL_POSE);

            new SequentialCommandGroup(
                        // Obelisk and MoulinCalibrate
                        new InitToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem,
                                CLOSE_SHOOT_BLUE_POSE, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY),

                        //SECOND ROW FIRST
                        new ShootToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                SECOND_ROW_BLUE_POSE, CLOSE_SHOOT_BLUE_POSE, new InstantCommand(),CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY,
                                LENGTH_X_ROW_SUPER_23RD, SUPER_POWER_ROW_PICK_ARTEFACTS, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        new ShootToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, BLUE_RAMP_POSE, BLUE_RAMP_END_POSE, CLOSE_SHOOT_BLUE_POSE, new InstantCommand(), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        new ShootToGatePickToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem, BLUE_RAMP_POSE, BLUE_RAMP_END_POSE, CLOSE_SHOOT_BLUE_POSE, new InstantCommand(), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY, GATEPICK_POWER, LINEAR_HEADING_INTERPOLATION_END_TIME/1.5),

                        //FIRST ROW LAST
                        new ShootToRowToShoot(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, chargeurSubsystem, visionSubsystem, gamepadSubsystem,
                                FIRST_ROW_BLUE_POSE, CLOSE_SHOOT_BLUE_POSE, new InstantCommand(), CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY,
                                LENGTH_X_ROW_SUPER, SUPER_POWER_ROW_PICK_ARTEFACTS, LINEAR_HEADING_INTERPOLATION_END_TIME/1.8),

                        new ParallelCommandGroup(
                                new StopChargeur(chargeurSubsystem),
                                new StopShooter(shooterSubsystem))

            ).schedule();
    }

    /**
     * Main OpMode loop. Updates gamepad states.
     */
    @Override
    public void run() {
            super.run();
    }


}
