package org.firstinspires.ftc.teamcode.odometry.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

public class Drivetrain {

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    HardwareMap hardwaremap;
    /**
     * Mecanum divetrain
     * FL-------FR
     * |        |
     * |        |
     * |        |
     * BL------BR
     */

    DcMotorEx motorFrontLeft; //FL in picture above
    DcMotorEx motorFrontRight;//FR in picture above
    DcMotorEx motorBackLeft;//BL in picture above
    DcMotorEx motorBackRight;//BR in picture above



    public static DcMotor encoderLeft;
    public static DcMotor encoderRight;
    public static DcMotor encoderMiddle;

    public Drivetrain(HardwareMap hardwaremap){
        this.hardwaremap = hardwaremap;

        imu = hardwaremap.get(BNO055IMU.class, "imu");

        /**
         * Make sure the {deviceName} is set to the right name in your hardware map
         */

        motorFrontLeft = hardwaremap.get(DcMotorEx.class, "frontLeft");
        motorFrontRight = hardwaremap.get(DcMotorEx.class, "frontRight");
        motorBackLeft = hardwaremap.get(DcMotorEx.class, "backLeft");
        motorBackRight = hardwaremap.get(DcMotorEx.class, "backRight");

        /**
         * Reverse some of your motors.
         */
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE); //dit klopt wanneer de engineers capable zijn; dus je moet waarschijnlijk dingen aanpassen.
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Initializing motors:", "Done");

        /**
         * The encoders in the hardware map
         */
        encoderLeft = hardwaremap.get(DcMotorEx.class, "encoderLeft");
        encoderRight = hardwaremap.get(DcMotorEx.class, "encoderRight");
        encoderMiddle = hardwaremap.get(DcMotorEx.class, "encoderMiddle");

    }

    public void drive(double gamepadXCoordinate, double gamepadYCoordinate, double gamepadHypot){
        double driveTurn;
        //double driveVertical;
        //double driveHorizontal;

        //double gamepadXCoordinate;
        //double gamepadYCoordinate;
        //double gamepadHypot; //speed
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /**
         * make sure you've configured your imu properly and with the correct device name
         */

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        driveTurn = -gamepad1.left_stick_x;

        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        /**
         * gamepadXCoordinate = gamepad1.left_stick_x; // gives our x value relative to the driver
         * gamepadYCoordinate = -gamepad1.left_stick_y; // gives our y value relative to the driver
         * gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
         **/
        //gamepadHypot finds how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
        gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotDegree = getAngle();
        //gives us the angle our robot is at
        movementDegree = gamepadDegree - robotDegree;
        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors

        //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
        motorFrontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        motorBackRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        motorFrontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        motorBackLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
