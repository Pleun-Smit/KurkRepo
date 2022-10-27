package org.firstinspires.ftc.teamcode.odometry.Util.geometry;

/**
 * Pose2d is a data container that stores x and y components of the robot position in meters
 * and the rotation in radians (CCW is positive).
 */
public class Pose2d {
    private double x;
    private double y;
    private double rotation;

    /**
     * Creates a new Pose2d object with given coordinates and rotation. When using field space
     * coordinates, the blue terminal on the audience side is considered (0,0).
     *
     * @param xMeters distance from the blue alliance wall in meters
     * @param yMeters distance from the audience wall in meters
     * @param rotationRadians angle in radians the red alliance station and the robot
     */
    public Pose2d(double xMeters, double yMeters, double rotationRadians) {
        x = xMeters;
        y = yMeters;
        rotation = rotationRadians;
    }

    /**
     * Gets the distance from the blue alliance wall in meters
     *
     * @return the distance from the blue alliance wall in meters
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the distance from the audience wall in meters
     *
     * @return the distance from the audience wall in meters
     */
    public double getY() {
        return y;
    }

    /**
     * Gets the angle in radians between the vector facing the red alliance wall and the
     * forward vector of the robot. A positive angle is counterclockwise
     *
     * @return angle in radians between the red alliance wall and the forward vector of the robot
     */
    public double getRotation() {
        return rotation;
    }

    /**
     * Subtracts the {@code targetPose} from the current pose.
     *
     * @param targetPose the {@code Pose2d} to subtract from this {@code Pose2d}
     * @return the difference between this pose and the target pose
     */
    public Pose2d minus(Pose2d targetPose) {
        double deltaX = x - targetPose.getX();
        double deltaY = y - targetPose.getY();
        double deltaR = rotation - targetPose.getRotation();

        return new Pose2d(deltaX, deltaY, deltaR);
    }

    /**
     * Adds the {@code targetPose} from the current pose.
     *
     * @param targetPose the {@code Pose2d} to add to this {@code Pose2d}
     * @return the addition of this pose and the target pose
     */
    public Pose2d plus(Pose2d targetPose) {
        double sumX = x + targetPose.getX();
        double sumY = y + targetPose.getY();
        double sumR = rotation + targetPose.getRotation();

        return new Pose2d(sumX, sumY, sumR);
    }
}