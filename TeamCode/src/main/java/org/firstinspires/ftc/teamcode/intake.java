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

// This is the class for all the calling and organizing of the functions. It runs the logic on the WHOLE
// teleop, not the specific "buttons doing what" stuff, but deciding what should and shouldn't run
// based off of input from the specific classes.
@TeleOp(name = "intake", group = "Teleops")
@Config
public class intake extends OpMode {

    DcMotor intakeSpinners;
    DcMotor intakeSpinners2;


    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init(){

        intakeSpinners = hardwareMap.dcMotor.get("its");
        intakeSpinners2 = hardwareMap.dcMotor.get("itss");
        intakeSpinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinners2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {
        if (gamepad2.right_trigger > 0.4)
        {
            intakeSpinners.setPower(1);
            intakeSpinners2.setPower(-1);
        }
        else if (gamepad2.left_trigger == 0)
        {
            intakeSpinners.setPower(0);
            intakeSpinners2.setPower(0);
        }
        else if (gamepad2.left_trigger > 0.4)
        {
            intakeSpinners.setPower(-1);
            intakeSpinners2.setPower(1);        }

    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
