package org.firstinspires.ftc.teamcode.odometry.auto.modes;

import static org.firstinspires.ftc.teamcode.odometry.Constants.OMNI_TICKS_TO_METERS;
import static org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain.encoderLeft;
import static org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain.encoderRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.Util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain;

@TeleOp
public class OdoTest extends OpMode {
    private BNO055IMU imu;
    private DcMotor encoderX;
    private DcMotor encoderY1;
    private DcMotor encoderY2;

    private Pose2d currentPose = new Pose2d(0,0,0);

    private double previousHeading;

    /**
     * Creates a new Odometry object.
     *
     * //@param hardwareMap the current {@code HardwareMap} of the robot
     */
    public void init() {
            // TODO get adresses from hardware map
            //Drivetrain drive = new Drivetrain(hardwareMap);

            encoderY1 = hardwareMap.get(DcMotor.class, "encoderY1");
            encoderY2 = hardwareMap.get(DcMotor.class, "encoderY2");

            encoderX = hardwareMap.get(DcMotor.class, "encoderX");

            // TODO configure IMU
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
            // TODO set conversion factors for encoders
        }

        public void loop() {
            updatePose();
            telemetry.addData("ticks x encoder", encoderX.getCurrentPosition());
            telemetry.addData("ticks y", encoderY1.getCurrentPosition());
            telemetry.addData("ticks y2", encoderY2.getCurrentPosition());

            telemetry.addLine("Current Pose X = " + currentPose.getX());
            telemetry.update();
        }

    /**
     * Updates the current {@code Pose2d} of the robot. For accurate results, run every loop.
     */
    public void updatePose() {
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentHeading = imu.getPosition().z; //als alles kurk is Z veranderen naar Y

        double averageY = (encoderY1.getCurrentPosition() + encoderY2.getCurrentPosition()) / 2;

        // TODO calculate distance traveled X and Y
        double deltaRightMeters = (encoderX.getCurrentPosition() * OMNI_TICKS_TO_METERS);
        double deltaForwardMeters = (averageY * OMNI_TICKS_TO_METERS);
        double deltaAngleRadians = currentHeading - previousHeading;

        // Create vectors (Pose2ds) representations for each displacement
        Pose2d vectorRightMeters = new Pose2d(
                Math.cos(currentHeading) * deltaRightMeters,
                Math.sin(currentHeading) * deltaRightMeters,
                deltaAngleRadians
        );

        Pose2d vectorUpMeters = new Pose2d(
                Math.sin(currentHeading) * deltaForwardMeters,
                Math.cos(currentHeading) * deltaForwardMeters,
                0
        );

        currentPose = currentPose.plus(vectorRightMeters).plus(vectorUpMeters);


        // reset encoder values

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * Gets the current {@code Pose2d} of the robot
     *
     * @return current {@code Pose2d} of the robot
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }
}