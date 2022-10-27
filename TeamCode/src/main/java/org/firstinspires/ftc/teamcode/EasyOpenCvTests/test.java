package org.firstinspires.ftc.teamcode.EasyOpenCvTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.utility.Globalvalues;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config //Comment out if not using FTC Dashboard
@Autonomous(name="Multiple colors test open cv", group="Tutorials")

public class test extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    //pink
    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    //yellow
    private double CrLowerUpdate2 = 100;
    private double CbLowerUpdate2 = 0;
    private double CrUpperUpdate2 = 170;
    private double CbUpperUpdate2 = 120;

    //green
    private double CrLowerUpdate3 = 100;
    private double CbLowerUpdate3 = 0; //131, 90, 82
    private double CrUpperUpdate3 = 90;
    private double CbUpperUpdate3 = 82;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    //yellow
    //private double lowerruntime2 = 0;
    //private double upperruntime2 = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
    public static Scalar scalarLowerYCrCb2 = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCb2 = new Scalar(255.0, 170.0, 120.0);

    //Green Range
    public static Scalar scalarLowerYCrCb3 = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCb3 = new Scalar(255, 90, 82);

    @Override
    public void runOpMode()
    {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        Pipeline1 myPipeline;
        webcam.setPipeline(myPipeline = new Pipeline1(borderLeftX,borderRightX,borderTopY,borderBottomY));

        // Configuration of Pipeline
        //pink
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        //yellow
        myPipeline.configureScalarLower2(scalarLowerYCrCb2.val[0],scalarLowerYCrCb2.val[1],scalarLowerYCrCb2.val[2]);
        myPipeline.configureScalarUpper2(scalarUpperYCrCb2.val[0],scalarUpperYCrCb2.val[1],scalarUpperYCrCb2.val[2]);

        //green
        myPipeline.configureScalarLower3(scalarLowerYCrCb3.val[0],scalarLowerYCrCb3.val[1],scalarLowerYCrCb3.val[2]);
        myPipeline.configureScalarUpper3(scalarUpperYCrCb3.val[0],scalarUpperYCrCb3.val[1],scalarUpperYCrCb3.val[2]);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only when you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    AUTONOMOUS_C();
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    AUTONOMOUS_B();
                }
                else {
                    AUTONOMOUS_A();
                }
            }


        }
    }
    public void testing(Pipeline1 myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();

            //yellow


        }
        //pink
        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        //yellow
        CrLowerUpdate2 = inValues2(CrLowerUpdate2, 0, 255);
        CrUpperUpdate2 = inValues2(CrUpperUpdate2, 0, 255);
        CbLowerUpdate2 = inValues2(CbLowerUpdate2, 0, 255);
        CbUpperUpdate2 = inValues2(CbUpperUpdate2, 0, 255);

        //green
        CrLowerUpdate3 = inValues3(CrLowerUpdate3, 0, 255);
        CrUpperUpdate3 = inValues3(CrUpperUpdate3, 0, 255);
        CbLowerUpdate3 = inValues3(CbLowerUpdate3, 0, 255);
        CbUpperUpdate3 = inValues3(CbUpperUpdate3, 0, 255);

        //pink
        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        //yellow
        myPipeline.configureScalarLower2(0.0, CrLowerUpdate2, CbLowerUpdate2);
        myPipeline.configureScalarUpper2(255.0, CrUpperUpdate2, CbUpperUpdate2);

        //green
        myPipeline.configureScalarLower3(0.0, CrLowerUpdate3, CbLowerUpdate3);
        myPipeline.configureScalarUpper3(255.0, CrUpperUpdate3, CbUpperUpdate3);

        //pink
        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);

        //yellow
        telemetry.addData("lowerCr2 ", (int)CrLowerUpdate2);
        telemetry.addData("lowerCb2 ", (int)CbLowerUpdate2);
        telemetry.addData("UpperCr2 ", (int)CrUpperUpdate2);
        telemetry.addData("UpperCb2 ", (int)CbUpperUpdate2);

        //green
        telemetry.addData("lowerCr3 ", (int)CrLowerUpdate3);
        telemetry.addData("lowerCb3 ", (int)CbLowerUpdate3);
        telemetry.addData("UpperCr3 ", (int)CrUpperUpdate3);
        telemetry.addData("UpperCb3 ", (int)CbUpperUpdate3);
    }

    //pink
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    //yellow
    public Double inValues2(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    //green
    public Double inValues3(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }

    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
    }
}