package org.firstinspires.ftc.teamcode.Subsystems;

//import com.arcrobotics.ftclib2.command.SubsystemBase;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class ExampleSubsystem extends SubsystemBase {

    private static Servo mechRotation;

    public ExampleSubsystem(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public static void grab() {
        telemetry.addLine("grab");
        telemetry.update();
        //mechRotation.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public static void release() {
        mechRotation.setPosition(0);
    }

    public static void test(){
        telemetry.addLine("test");
        telemetry.update();
    }

    public static void test2(){
        telemetry.addLine("test2");
        telemetry.update();
    }

}