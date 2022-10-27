package org.firstinspires.ftc.teamcode.odometry.Kinematics;

public class ChassisSpeeds {
    private double x;
    private double y;
    private double rotation;

    /**
     * Creates a new ChassisSpeeds object with given speeds. When using field space
     * coordinates, the blue terminal on the audience side is considered (0,0).
     *
     * @param xMeters velocity towards the blue alliance wall in m/s
     * @param yMeters velocity towards the audience wall in m/s
     * @param rotationRadians angular velocity in rad/s
     */
    public ChassisSpeeds(double xMeters, double yMeters, double rotationRadians) {
        x = xMeters;
        y = yMeters;
        rotation = rotationRadians;
    }

    /**
     * Gets the velocity towards the blue alliance wall in m/s
     *
     * @return the velocity towards the blue alliance wall in m/s
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the velocity towards the audience wall in m/s
     *
     * @return the velocity towards the audience wall in m/s
     */
    public double getY() {
        return y;
    }

    /**
     * Gets the angular velocity of the robot. A positive angle is counterclockwise
     *
     * @return angle in radians between the red alliance wall and the forward vector of the robot
     */
    public double getRotation() {
        return rotation;
    }
}