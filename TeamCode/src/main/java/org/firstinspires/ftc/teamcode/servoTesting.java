package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "servoTesting", group = "Teleops")
@Config
public class servoTesting extends OpMode {


    // outtake servos
    Servo curServo;
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    double rotatorConstant = 0;

    // right bumper - add 0.1 to the servo
    // left bumper - subtract 0.1 to the servo

////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void init() {
        curServo = hardwareMap.servo.get(("curServo"));
    }

    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        telemetry.addData("Servo position", rotatorConstant);
        telemetry.update();
        if (gamepad2.right_bumper && !previousGamepad2.right_bumper)
        {
            rotatorConstant += 0.1;
            telemetry.update();
        }
        else if (gamepad2.left_bumper && !previousGamepad2.left_bumper)
        {
            rotatorConstant -= 0.1;
            telemetry.update();
        }
        curServo.setPosition(rotatorConstant);

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
