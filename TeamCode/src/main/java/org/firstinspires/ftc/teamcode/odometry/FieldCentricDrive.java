package org.firstinspires.ftc.teamcode.odometry;

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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

/**
 * Currently testing how to use methods from other classes
 */
@TeleOp(name = "DriverRelativeControl", group = "tutorial")
public class FieldCentricDrive extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain(hardwareMap);

    double gamepadXcoordinate = gamepad1.left_stick_x;
    double gamepadYcoordinate = gamepad1.left_stick_y;
    double gamepadHypod = Range.clip(Math.hypot(gamepadXcoordinate, gamepadYcoordinate), 0, 1);

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {
            drivetrain.drive(gamepadXcoordinate, gamepadYcoordinate, gamepadHypod);
        }
    }
}
