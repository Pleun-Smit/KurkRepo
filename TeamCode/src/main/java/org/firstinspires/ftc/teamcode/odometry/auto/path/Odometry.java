package org.firstinspires.ftc.teamcode.odometry.auto.path;

import static org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain.encoderLeft;
import static org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain.encoderRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.odometry.Util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain;

public class Odometry {
    private BNO055IMU imu;
    private DcMotor encoderX;
    private DcMotor encoderY;

    private Pose2d CurrentPose;

    //Position variables used for storage and calculations
    double leftEncoderPos = 0, rightEncoderPos = 0, middleEncoderPos = 0;
    private double previousRightEncoderPos = 0, previousLeftEncoderPos = 0;

    /**
     * The multiplier is calculated by "measured distance/telemetry distance"
     */
    private int xEncoderPositionMultiplier = 1;
    private int yEncoderPositionMultiplier = 1;

    private Pose2d currentPose;
    private Pose2d newPose;


    //private Pose2d currentPoseY;
    //private Pose2d newPoseY;

    /**
     * Creates a new Odometry object.
     *
     * @param hardwareMap the current {@code HardwareMap} of the robot
     */
    public Odometry(HardwareMap hardwareMap) {
        // TODO get adresses from hardware map
        Drivetrain drive = new Drivetrain(hardwareMap);


        // TODO configure IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    /**
     * Updates the current {@code Pose2d} of the robot. For accurate results, run every loop.
     *
     * getCurrentPosition() {@code getCurrentPosition} is in ticks so you need to convert it
     */
    public void updatePose() {
        // TODO calculate distance traveled X and Y
        //Gets the current Pos
        leftEncoderPos = (encoderLeft.getCurrentPosition() * xEncoderPositionMultiplier);
        rightEncoderPos = (encoderRight.getCurrentPosition() * xEncoderPositionMultiplier);

        //Gets the change in position
        double leftChange = leftEncoderPos - previousLeftEncoderPos;
        double rightChange = rightEncoderPos - previousRightEncoderPos;


        // TODO create new pose based on distance traveled + old pose
        double newPoseLeft = leftChange + previousLeftEncoderPos;
        double newPoseRight = rightChange + previousRightEncoderPos; //distance traveled + old pose

        // TODO set new pose to currentPose
        newPose = currentPose;


    }

    /**
     * Gets the current {@code Pose2d} of the robot
     *
     * @return current {@code Pose2d} of the robot
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public Pose2d getNewPose(){
        return newPose;
    }
}