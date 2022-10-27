package org.firstinspires.ftc.teamcode.odometry;

/**
 * Java class with al constant variables
 */

public class Constants {

    public static double Wheel_Radius = 4.800092; // cm
    public static double Gear_Ratio = 1; // output (wheel) speed / input (motor) speed
    public static double Track_Width = 29.000000001; // cm
    public static final double Ticks_Per_Rev = 383.6;

    // TODO set conversion factors for encoders
    public static double encoderTicksToMeters(double ticks) {
        return Wheel_Radius * 2 * Math.PI * Gear_Ratio * ticks / Ticks_Per_Rev / 100;
    }


    /* Nigel conversion */
    public static double OMNI_TICKS_PER_REV = 383.6;
    public static double OMNI_WHEEL_RADIUS_METERS = 0.05;
    public static double OMNI_METERS_PER_REV = 2 * Math.PI * OMNI_WHEEL_RADIUS_METERS;
    public static double OMNI_TICKS_TO_METERS = OMNI_METERS_PER_REV/ OMNI_TICKS_PER_REV;
}
