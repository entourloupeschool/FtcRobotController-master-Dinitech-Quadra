package org.firstinspires.ftc.teamcode.dinitech.other;

import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.LAUNCH_ZONE_BIG;
import static org.firstinspires.ftc.teamcode.dinitech.other.FieldDefinitions.LAUNCH_ZONE_SMALL;
import static org.firstinspires.ftc.teamcode.dinitech.other.RobotDefinitions.ROBOT_LENGTH_INCH;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem.MIN_LINEAR;

import com.bylazar.configurables.annotations.Configurable;
@Configurable
public class Globals {
    public static final class Vec2 {
        public final double x;
        public final double y;

        public Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vec2 add(Vec2 other) {
            return new Vec2(x + other.x, y + other.y);
        }

        public Vec2 subtract(Vec2 other) {
            return new Vec2(x - other.x, y - other.y);
        }

        public double dot(Vec2 other) {
            return x * other.x + y * other.y;
        }

        public double magnitude() {
            return Math.sqrt(x * x + y * y);
        }

        public Vec2 normalize() {
            double mag = magnitude();
            return new Vec2(x / mag, y / mag);
        }

        public Vec2 multiply(double scalar) {
            return new Vec2(x * scalar, y * scalar);
        }

        public double euclidianDistance(Vec2 other) {
            return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
        }
    }




    /**
     * Gets the closest center target so the robot can face the basket and still touch a launch zone.
     *
     * The robot center is intentionally kept outside the zone: we find the closest point on the selected
     * launch zone edge, then back off by half robot length plus a safety margin in the opposite direction
     * of the basket-facing vector.
     */
    public static Vec2 getClosestVec2InLaunchZone(Vec2 point, Vec2 basketPoint, int lauchZoneNumber) {
        Vec2 closest = null;
        double minDist = Double.MAX_VALUE;

        if ((lauchZoneNumber == 1 || lauchZoneNumber == 3) && isPointInsideTriangle(point, LAUNCH_ZONE_BIG)) {
            return point;
        }
        if ((lauchZoneNumber == 2 || lauchZoneNumber == 3) && isPointInsideTriangle(point, LAUNCH_ZONE_SMALL)) {
            return point;
        }

        if (lauchZoneNumber == 1 || lauchZoneNumber == 3) {
            for (int i = 0; i < LAUNCH_ZONE_BIG.length; i++) {
                Vec2 a = LAUNCH_ZONE_BIG[i];
                Vec2 b = LAUNCH_ZONE_BIG[(i + 1) % LAUNCH_ZONE_BIG.length];
                Vec2 closestOnEdge = getClosestPointOnLineSegment(point, a, b);
                double dist = point.euclidianDistance(closestOnEdge);
                if (dist < minDist) {
                    minDist = dist;
                    closest = closestOnEdge;
                }
            }
        }

        if (lauchZoneNumber == 2 || lauchZoneNumber == 3) {
            for (int i = 0; i < LAUNCH_ZONE_SMALL.length; i++) {
                Vec2 a = LAUNCH_ZONE_SMALL[i];
                Vec2 b = LAUNCH_ZONE_SMALL[(i + 1) % LAUNCH_ZONE_SMALL.length];
                Vec2 closestOnEdge = getClosestPointOnLineSegment(point, a, b);
                double dist = point.euclidianDistance(closestOnEdge);
                if (dist < minDist) {
                    minDist = dist;
                    closest = closestOnEdge;
                }
            }
        }

        if (closest == null) {
            return point;
        }

        Vec2 basketDirection = basketPoint.subtract(closest);
        double basketDirectionMagnitude = basketDirection.magnitude();
        if (basketDirectionMagnitude < 1e-6) {
            return closest;
        }

        Vec2 headingToBasket = basketDirection.multiply(1.0 / basketDirectionMagnitude);
        double centerBackoff = ROBOT_LENGTH_INCH / 2.0 + LAUNCH_ZONE_TOUCH_MARGIN_INCH;

        return closest.subtract(headingToBasket.multiply(centerBackoff));
    }

    private static double area(Vec2 a, Vec2 b, Vec2 c) {
        return Math.abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
    }

    private static boolean isPointInsideTriangle(Vec2 p, Vec2[] triangle) {
        double areaABC = area(triangle[0], triangle[1], triangle[2]);
        double areaPAB = area(p, triangle[0], triangle[1]);
        double areaPBC = area(p, triangle[1], triangle[2]);
        double areaPCA = area(p, triangle[2], triangle[0]);

        return areaPAB + areaPBC + areaPCA <= areaABC + 1e-3;
    }

    private static Vec2 getClosestPointOnLineSegment(Vec2 p, Vec2 a, Vec2 b) {
        Vec2 ab = b.subtract(a);
        double t = (p.subtract(a)).dot(ab) / ab.dot(ab);
        t = Math.max(0, Math.min(1, t));
        return a.add(ab.multiply(t));
    }

    public static double LAUNCH_ZONE_TOUCH_MARGIN_INCH = 5.0;


        public static double pickCustomPowerFunc(double x, int funcNumber) {
                switch (funcNumber) {
                    case 2:
                            return customPowerFunc2(x);
                    case 3:
                            return customPowerFunc3(x);
                    case 4:
                            return customPowerFunc4(x);
                    case 5:
                            return customLinearFunc(x);
                    case 6:
                        return customPowerFunc5(x);
                    default:
                            return customPowerFunc(x);
                }
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.5;
                double shifted3 = shifted * shifted * shifted; // Faster than Math.pow(shifted, 3)

                return Math.signum(x) * (3 * shifted3 + absValue / 4 + 0.375);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc2(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.6;
                // Manually expand pow(shifted, 5) = shifted * shifted * shifted * shifted *
                // shifted
                double shifted2 = shifted * shifted;
                double shifted4 = shifted2 * shifted2;
                double shifted5 = shifted4 * shifted;

                return Math.signum(x) * (8.1644718783 * shifted5 + absValue / 3.551975 + 0.6538649554);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc3(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.65;
                double shifted2 = shifted * shifted;
                double shifted4 = shifted2 * shifted2;
                double shifted5 = shifted4 * shifted;

                return Math.signum(x)
                                * (7.6704545455 * shifted5 + absValue / 14.08 + 0.8920454545);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc4(double x) {
                double absValue = Math.abs(x);
                double shifted = absValue - 0.56;
                double shifted3 = shifted * shifted * shifted;

                return Math.signum(x)
                                * (1.4705882353 * shifted3 + absValue * 0.5882352941 + 0.2647058824);
        }

        /**
         * Applies a custom function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customPowerFunc5(double x) {
            double absValue = Math.abs(x);
            double shifted = absValue - 0.405;
            double shifted7 = shifted * shifted * shifted * shifted * shifted * shifted * shifted;

            return Math.signum(x)
                    * (224.22 * shifted7 + absValue * 0.07407407407 + 0.4192666667);
        }

        /**
         * Applies a custom Linear function while preserving the sign of the input
         *
         * @param x The input value
         * @return The custom-transformed value with preserved sign
         */
        public static double customLinearFunc(double x) {
                if (Math.abs(x) < MIN_LINEAR) {
                        return 0;
                }
                return Math.signum(x) * (x * (1 - MIN_LINEAR) + MIN_LINEAR);
        }

        public static double cmToInch(double cm){
            return cm * 0.3937007874;
        }
        public static double inchToCm(double inch){
                return inch * 2.54;
        }
        public static double mmToInch(double cm){
            return cmToInch(cm) * 0.1;
        }


    /**
     * A zero-allocation circular buffer for calculating a running average.
     */
    public static class RunningAverage {
        private final double[] samples;
        private int index = 0;
        private int count = 0;
        private double runningSum = 0;

        public RunningAverage(int size) {
            this.samples = new double[size];
        }

        public void add(double value) {
            runningSum -= samples[index];
            samples[index] = value;
            runningSum += value;
            index = (index + 1) % samples.length;
            if (count < samples.length) {
                count++;
            }
        }

        public Double getAverage() {
            return count == 0 ? null : runningSum / count;
        }

        public double getStd() {
            if (count == 0) return 0;
            double mean = getAverage();
            double sumSquaredDiffs = 0;
            for (int i = 0; i < count; i++) {
                double diff = samples[i] - mean;
                sumSquaredDiffs += diff * diff;
            }
            return Math.sqrt(sumSquaredDiffs / count);
        }

        public boolean isEmpty() {
            return count == 0;
        }

        public int size() {
            return count;
        }
    }
}
