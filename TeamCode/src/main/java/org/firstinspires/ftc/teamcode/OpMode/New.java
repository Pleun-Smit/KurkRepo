package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class New extends OpMode {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;


//


    double MotorFrontLeftPower;
    double MotorFrontRightPower;
    double MotorBackLeftPower;
    double MotorBackRightPower;

    @Override
    public void init() {



        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        //check config on phone for names dc motors

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //servoIntake.setPosition(-40);







    }
    @Override
    public void loop() {
        //servoIntake.setPosition(50);





        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately

        double speed = 0.65;
        double slowspeed = 0.5;
        // Update everything. Odometry. Etc.

        motorFrontLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)* speed);
        motorFrontRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)* speed);
        motorBackLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)* speed);
        motorBackRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)* speed);

        //formulas ^


        //motorFrontLeft.setPower(MotorFrontLeftPower);
        //motorFrontRight.setPower(MotorFrontRightPower);
        //motorBackLeft.setPower(MotorBackLeftPower);
        //motorBackRight.setPower(MotorBackRightPower);



    }
}
