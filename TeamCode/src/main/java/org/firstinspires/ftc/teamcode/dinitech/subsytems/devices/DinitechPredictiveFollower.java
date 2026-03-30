package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem.PAR_POD_Y_MM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem.PERP_POD_X_MM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
public class DinitechPredictiveFollower {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.05, 0.03, 0.012))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.25, 0.1153178675, 0.0022423725))
            .centripetalScaling(0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}