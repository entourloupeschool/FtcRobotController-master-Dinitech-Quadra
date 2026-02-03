package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromPedroRange;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.function.BooleanSupplier;

public class PedroShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final BooleanSupplier isBlueSupplier;

    private boolean lastIsBlue = false;

    private Pose targetBasketPose;



    /**
     * Creates a PedroShooter command that adjusts shooter speed based on the localizer's pose's distance to basket
     * range.
     *
     * @param shooterSubsystem The shooter subsystem
     * @param drivePedroSubsystem  The drive subsystem
     * @param isBlueSupplier wether the robot is on blue team or not
     */
    public PedroShooter(ShooterSubsystem shooterSubsystem, DrivePedroSubsystem drivePedroSubsystem, BooleanSupplier isBlueSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.isBlueSupplier = isBlueSupplier;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.PEDRO);
        targetBasketPose = BLUE_BASKET_POSE;
    }


    @Override
    public void execute() {
        if (drivePedroSubsystem.getDriverInputPose()){
            // Get current pose from drive
            Pose currentPose = drivePedroSubsystem.getPose();
            boolean currentIsBlue = isBlueSupplier.getAsBoolean();

            // Get distance to blue basket if blue team, else get distance to red basket
            if (currentIsBlue != lastIsBlue) {
                lastIsBlue = currentIsBlue;
                targetBasketPose = currentIsBlue ? BLUE_BASKET_POSE : RED_BASKET_POSE;
            }

            // Set shooter speed based on distance to basket
            shooterSubsystem.setVelocity(linearSpeedFromPedroRange(currentPose.distanceFrom(targetBasketPose)));
        }

    }


}
