package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_START_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BRAKING_STRENGTH_PEDRO_DINITECH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PAR_POD_Y_MM;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PERP_POD_X_MM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class DinitechFollower {
    public static double kF = 0.012;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-52.3857)
            .lateralZeroPowerAcceleration(-100.4664)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.55, 0.007, 0.03, kF))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25, 0.0001, 0.0001, kF))
            .useSecondaryTranslationalPIDF(false)

            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.005, 0.03, kF))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3, 0.06, 0.15, kF))
            .useSecondaryHeadingPIDF(false)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.11,0.001,0.012,0.3, kF))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0045,0.0003,0.0002,0.3, kF))
            .useSecondaryDrivePIDF(false)

            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.2)
            .yVelocity(67.35);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(PAR_POD_Y_MM)  // https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
            .strafePodX(PERP_POD_X_MM)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(ENCODER_RESOLUTION) //through bore encoder https://www.revrobotics.com/rev-11-1271/ wheels: https://www.revrobotics.com/ION-Omni-Wheels/ // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/specs
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, BRAKING_STRENGTH_PEDRO_DINITECH, BRAKING_START_PEDRO_DINITECH);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}