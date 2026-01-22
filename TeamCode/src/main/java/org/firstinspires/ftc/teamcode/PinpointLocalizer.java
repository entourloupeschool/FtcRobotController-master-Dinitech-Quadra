package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PAR_POD_Y_MM;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PERP_POD_X_MM;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 425233.41728004813; //423407.4722914854; //415418.07405333535; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 474571.18565781455; //449193.529173382; //423307.79222261213; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver pinPointDriver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        pinPointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4; // = 0.01940346363
//        pinPointDriver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
//        pinPointDriver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        pinPointDriver.setEncoderResolution(ENCODER_RESOLUTION, DistanceUnit.MM);
        pinPointDriver.setOffsets(PAR_POD_Y_MM, PERP_POD_X_MM, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        pinPointDriver.setEncoderDirections(initialParDirection, initialPerpDirection);

        pinPointDriver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        pinPointDriver.update();
        if (Objects.requireNonNull(pinPointDriver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(pinPointDriver.getPosX(DistanceUnit.INCH), pinPointDriver.getPosY(DistanceUnit.INCH), pinPointDriver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(pinPointDriver.getVelX(DistanceUnit.INCH), pinPointDriver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, pinPointDriver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
