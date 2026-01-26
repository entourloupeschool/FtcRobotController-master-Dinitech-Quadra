package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PAR_POD_Y_MM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PATH_BUILDER_FORWARD_ZERO_POWER_ACCELERATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PATH_BUILDER_LATERAL_ZERO_POWER_ACCELERATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PERP_POD_X_MM;

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

public class DinitechFollower {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-52.3857 * PATH_BUILDER_FORWARD_ZERO_POWER_ACCELERATION)
            .lateralZeroPowerAcceleration(-100.4664 * PATH_BUILDER_LATERAL_ZERO_POWER_ACCELERATION)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.18, 1.0E-4, 0.02, 1.0E-4))
            .headingPIDFCoefficients(new PIDFCoefficients(1.35, 0.03, 0.01, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.01,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.000005,0.6,0.01))
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
            .forwardPodY(PAR_POD_Y_MM)
            .strafePodX(PERP_POD_X_MM)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(ENCODER_RESOLUTION) //through bore encoder https://www.revrobotics.com/rev-11-1271/ wheels: https://www.revrobotics.com/ION-Omni-Wheels/ // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/specs
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}