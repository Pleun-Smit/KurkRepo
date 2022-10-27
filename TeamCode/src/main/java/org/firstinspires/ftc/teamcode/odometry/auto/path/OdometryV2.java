package org.firstinspires.ftc.teamcode.odometry.auto.path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.teamcode.odometry.Util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain;


public class OdometryV2 {
    private BNO055IMU imu;
    private DcMotorEx encoderX;
    private DcMotorEx encoderY;

    private Pose2d currentPose;
    double deltaPrevPose;
    double prevPose;

    private double x;
    private double y;
    private double heading;



    /**
     * Creates a new Odometry object.
     *
     * @param hardwareMap the current {@code HardwareMap} of the robot
     */
    public OdometryV2(HardwareMap hardwareMap) {
        // TODO get adresses from hardware map
        Drivetrain drive = new Drivetrain(hardwareMap);

        // TODO configure IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);





        // TODO set conversion factors for encoders
    }

    public double getHeading() {
        imu.getPosition();
        return heading;
    }

    /**
     * Updates the current {@code Pose2d} of the robot. For accurate results, run every loop.
     */
    public void updatePose() {
        // TODO calculate distance traveled X and Y

        // TODO create new pose based on distance traveled + old pose
         double newPose = prevPose + x + y;

        // TODO set new pose to currentPose


    }

    /**
     * Gets the current {@code Pose2d} of the robot
     *
     * @return current {@code Pose2d} of the robot
     */
    public Pose2d getCurrentPose() {
        x = Math.cos(heading) * deltaPrevPose;
        y = Math.sin(heading) * deltaPrevPose;
        return currentPose;
    }

}