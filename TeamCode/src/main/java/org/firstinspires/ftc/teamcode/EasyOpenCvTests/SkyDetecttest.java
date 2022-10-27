package org.firstinspires.ftc.teamcode.EasyOpenCvTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Skystone Detecotor", group="Auto")
public class SkyDetecttest extends LinearOpMode {
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        Pipeline1 myPipeline;
        // Configuration of Pipeline

        // Webcam Streaming
        SkystoneDetector detector = new SkystoneDetector(telemetry);


        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("Left");
                break;
            case RIGHT:
                telemetry.addLine("Right");
                break;
            case NOT_FOUND:
                telemetry.addLine("NOPE");
        }
        webcam.stopStreaming();
    }
}