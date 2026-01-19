package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AXIAL_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.AXIAL_VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.HEADING_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.HEADING_VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LATERAL_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.LATERAL_VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_ANGLE_ACCELERATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_ANGLE_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_PROFILE_ACCELERATION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_WHEEL_VELOCITY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MIN_PROFILE_ACCELERATION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * Represents the robot's mecanum drive system, based on the Road Runner library.
 * <p>
 * This class integrates hardware components (motors, IMU, voltage sensor) with Road Runner's
 * motion planning and control systems. It provides capabilities for both tele-operated control
 * and autonomous trajectory following.
 * <p>
 * Key components include:
 * <ul>
 *     <li>Hardware initialization and configuration.</li>
 *     <li>A {@link Localizer} for tracking the robot's position and orientation.</li>
 *     <li>Road Runner {@link Action} implementations for following trajectories and executing turns.</li>
 *     <li>Methods for creating trajectory sequences using a {@link TrajectoryActionBuilder}.</li>
 * </ul>
 */
@Config
public final class DinitechMecanumDrive {
    /**
     * Contains tunable parameters for the mecanum drive.
     */
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        private final double ticksTraveled = 360760.333;
        private final double distanceTraveled = 7 * 39.37007874; // 7 m * metersToInch = 275.59055118
        public final double inPerTick = distanceTraveled / ticksTraveled; // = 0.0007639158909;
        public final double lateralInPerTick = 0.00016933333;
        public final double trackWidthTicks = 1035974.7139795484;

        // feedforward parameters (in tick units)
        public double kS = 1.3908;
        public double kV = 0.0000932;
        public double kA = 0.000015;

        // path profile parameters (in inches)
        public double maxWheelVel = MAX_WHEEL_VELOCITY;
        public double minProfileAccel = MIN_PROFILE_ACCELERATION;
        public double maxProfileAccel = MAX_PROFILE_ACCELERATION;

        // turn profile parameters (in radians)
        public double maxAngVel = MAX_ANGLE_VELOCITY; // shared with path
        public double maxAngAccel = MAX_ANGLE_ACCELERATION;

        // path controller gains
        public double axialGain = AXIAL_GAIN;
        public double lateralGain = LATERAL_GAIN;
        public double headingGain = HEADING_GAIN; // shared with turn

        public double axialVelGain = AXIAL_VELOCITY_GAIN;
        public double lateralVelGain = LATERAL_VELOCITY_GAIN;
        public double headingVelGain = HEADING_VELOCITY_GAIN; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint = new MinVelConstraint(Arrays.asList(
            kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
            new AngularVelConstraint(PARAMS.maxAngVel)));
    public final AccelConstraint defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel,
            PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    /** The localizer used for pose estimation. Can be swapped at runtime for enhanced localization. */
    public Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**
     * A localizer that uses the drive motor encoders and the IMU for odometry.
     */
    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(DinitechMecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(DinitechMecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(DinitechMecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(DinitechMecanumDrive.this.rightFront));

            imu = lazyImu.get();

            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[] {
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[] {
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[] {
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[] {
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta));

            return twist.velocity().value();
        }
    }

    /**
     * Constructs a DinitechMecanumDrive with a PinpointLocalizer.
     * @param hardwareMap The robot's hardware map.
     * @param pose The initial pose of the robot.
     */
    public DinitechMecanumDrive(HardwareMap hardwareMap, Pose2d pose) {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /**
     * Sets the power of the drive motors based on a PoseVelocity2d.
     * @param powers The desired powers for the robot's movement (strafe, forward, turn).
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * Sets the velocity of the drive motors based on a PoseVelocity2d.
     * @param powers The desired velocities for the robot's movement (strafe, forward, turn).
     */
    public void setDriveVelocities(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setVelocity(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setVelocity(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setVelocity(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setVelocity(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * A Road Runner Action that follows a time-based trajectory.
     */
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * A Road Runner Action that executes a time-based turn.
     */
    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain)
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * Updates the robot's pose estimate using the localizer.
     * @return The robot's current velocity.
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * Creates a new {@link TrajectoryActionBuilder} to build and execute autonomous sequences.
     * @param beginPose The starting pose for the sequence.
     * @param constraintScale A scaling factor for the default constraints (0.0 to 1.0).
     * @return A new action builder.
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose, double constraintScale) {
        // Scale turn constraints
        TurnConstraints scaledTurnConstraints = new TurnConstraints(
                defaultTurnConstraints.maxAngVel * constraintScale,
                defaultTurnConstraints.minAngAccel * constraintScale,
                defaultTurnConstraints.maxAngAccel * constraintScale);
        // Scale velocity constraint (only wheel and angular constraints)
        VelConstraint scaledVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel * constraintScale),
                new AngularVelConstraint(PARAMS.maxAngVel * constraintScale)));
        // Scale accel constraint
        AccelConstraint scaledAccelConstraint = new ProfileAccelConstraint(
                PARAMS.minProfileAccel * constraintScale,
                PARAMS.maxProfileAccel * constraintScale);
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2)),
                beginPose, 0.0,
                scaledTurnConstraints,
                scaledVelConstraint, scaledAccelConstraint);
    }

    /**
     * Creates a new {@link TrajectoryActionBuilder} with default constraints.
     * @param beginPose The starting pose for the sequence.
     * @return A new action builder.
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return actionBuilder(beginPose, 1.0);
    }
}
