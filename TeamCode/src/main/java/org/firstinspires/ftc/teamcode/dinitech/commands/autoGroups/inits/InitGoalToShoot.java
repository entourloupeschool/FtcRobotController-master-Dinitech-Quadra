package org.firstinspires.ftc.teamcode.dinitech.commands.autoGroups.inits;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.HubsSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class InitGoalToShoot extends InitToShoot {

    public InitGoalToShoot(DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem){
        super(drivePedroSubsystem, trieurSubsystem, shooterSubsystem, CLOSE_SHOOT_AUTO_SHOOTER_VELOCITY);
    }

}
