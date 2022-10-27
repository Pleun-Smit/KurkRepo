package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odometry.subsystems.Drivetrain;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
/**
@TeleOp(group = "advanced")
@Disabled
public class FieldCenticV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        Drivetrain drive = new Drivetrain(hardwareMap);

        // TODO: Retrieve current pose from pose2d class

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // TODO: Read pose


            /** // Create a vector from the gamepad x/y inputs
             // Then, rotate that vector by the inverse of that heading
             Vector2d input = new Vector2d(
             -gamepad1.left_stick_y,
             -gamepad1.left_stick_x
             ).rotated(-poseEstimate.getHeading());

             // Pass in the rotated input + right stick value for rotation
             // Rotation is not part of the rotated input thus must be passed in separately
             drive.setWeightedDrivePower(
             new Pose2d(
             input.getX(),
             input.getY(),
             -gamepad1.right_stick_x
             )
             );

             // Update everything. Odometry. Etc.
             drive.update();

             // Print pose to telemetry
             telemetry.addData("x", poseEstimate.getX());
             telemetry.addData("y", poseEstimate.getY());
             telemetry.addData("heading", poseEstimate.getHeading());
             telemetry.update();
             **/