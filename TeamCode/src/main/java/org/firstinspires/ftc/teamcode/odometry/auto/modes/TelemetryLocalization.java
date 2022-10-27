package org.firstinspires.ftc.teamcode.odometry.auto.modes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.Util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odometry.auto.path.OdometryV3;

@TeleOp (group = "Test", name = "TeleLoc")
public class TelemetryLocalization extends OpMode {

    private BNO055IMU imu;
    private DcMotor encoderX;
    private DcMotor encoderY1;
    private DcMotor encoderY2;

    public void init() {
        encoderY1 = hardwareMap.get(DcMotor.class, "encoderY1");
        encoderY2 = hardwareMap.get(DcMotor.class, "encoderY2");

        encoderX = hardwareMap.get(DcMotor.class, "encoderX");

        // configure IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


    }

    public void loop() {
        OdometryV3 odometryV3 = new OdometryV3(hardwareMap);

        odometryV3.updatePose();
        Pose2d currentPose = odometryV3.getCurrentPose();


        telemetry.addLine("Current Pose X = " + currentPose.getX());
        telemetry.addLine("Current Pose Y = " + currentPose.getY());
        telemetry.addLine("Heading = " + currentPose.getRotation());

    }

}