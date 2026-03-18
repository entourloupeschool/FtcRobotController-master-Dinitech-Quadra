package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROBOT_LENGTH_INCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.ROBOT_WIDTH_INCH;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
public class DrawingDinitech {
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.35
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.35
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        double centerX = pose.getX();
        double centerY = pose.getY();
        double heading = pose.getHeading();

        double halfLength = ROBOT_LENGTH_INCH / 2.0;
        double halfWidth = ROBOT_WIDTH_INCH / 2.0;

        // Heading unit vector (front direction of the robot)
        double headingX = Math.cos(heading);
        double headingY = Math.sin(heading);

        // Left-normal unit vector for width direction
        double leftX = -headingY;
        double leftY = headingX;

        double frontLeftX = centerX + headingX * halfLength + leftX * halfWidth;
        double frontLeftY = centerY + headingY * halfLength + leftY * halfWidth;

        double frontRightX = centerX + headingX * halfLength - leftX * halfWidth;
        double frontRightY = centerY + headingY * halfLength - leftY * halfWidth;

        double rearRightX = centerX - headingX * halfLength - leftX * halfWidth;
        double rearRightY = centerY - headingY * halfLength - leftY * halfWidth;

        double rearLeftX = centerX - headingX * halfLength + leftX * halfWidth;
        double rearLeftY = centerY - headingY * halfLength + leftY * halfWidth;

        panelsField.setStyle(style);
        panelsField.moveCursor(frontLeftX, frontLeftY);
        panelsField.line(frontRightX, frontRightY);
        panelsField.moveCursor(frontRightX, frontRightY);
        panelsField.line(rearRightX, rearRightY);
        panelsField.moveCursor(rearRightX, rearRightY);
        panelsField.line(rearLeftX, rearLeftY);
        panelsField.moveCursor(rearLeftX, rearLeftY);
        panelsField.line(frontLeftX, frontLeftY);

        // Front heading indicator from center to front midpoint
        double frontMidX = centerX + headingX * halfLength;
        double frontMidY = centerY + headingY * halfLength;

        panelsField.setStyle(style);
        panelsField.moveCursor(centerX, centerY);
        panelsField.line(frontMidX, frontMidY);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}