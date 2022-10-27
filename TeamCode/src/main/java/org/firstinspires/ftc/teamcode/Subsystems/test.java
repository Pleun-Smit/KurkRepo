package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ExampleSubsystem;

import java.lang.reflect.Method;
@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() {
        ExampleSubsystem.grab();

        ExampleSubsystem.test();
    }
}
