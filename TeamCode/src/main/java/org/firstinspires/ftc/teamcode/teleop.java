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
@TeleOp(name = "TeleOp", group = "Teleops")
@Config
public class teleop extends OpMode {

    // Testing
    public static boolean testingActive = false;
    double lastTestTime = 0;

    ElapsedTime totalTime = new ElapsedTime();

    // Ease of controls stuff
    RRLocalizationRead localizationRead;

    DrivetrainControllers driveTrain;
    Sensors sensors;
    Mechanisms mechanisms;
    FileRead fileRead;
    ControllerHandler controllerHandler = new ControllerHandler();

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init(){


        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap, this);

        driveTrain = new DrivetrainControllers();
        driveTrain.initMotorsRR(this, localizationRead);

        controllerHandler = new ControllerHandler();
        controllerHandler.initController(this);

        sensors = new Sensors();
        sensors.init(this);

        mechanisms = new Mechanisms();
        mechanisms.init(this, driveTrain.frontLeftMotor, driveTrain.backRightMotor, driveTrain.frontRightMotor);

        fileRead = new FileRead();
        fileRead.init(this);


        mechanisms.initPastFirstFrame(controllerHandler);


        //localizationRead = new RRLocalizationRead();
        //localizationRead.initLocalization(hardwareMap);

    }

    public ControllerHandler getControllerHandler()
    {
        return controllerHandler;
    }

    // Driver 2 Controls:
    // a - transfer macro
    // y - toggle claw open and close
    // b - set outtake to neutral position in the middle of the slides so that it is out of the way
    // right trigger - spin intake in
    // left trigger - spin intake out
    // right stick - intake lift
    // left stick - outtake lift

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {

        //telemetry.addData("Pos: ", localizationRead.returnPose());
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.dpad_up && totalTime.milliseconds() > lastTestTime + 500)
        {
            if (testingActive)
                testingActive = false;
            else
                testingActive = true;
            lastTestTime = totalTime.milliseconds();
        }

        if (testingActive) {
            driveTrain.runTesting();
            //mechanisms.runTesting();
            //mechanisms.servotesting();
        }
        else {
            mechanisms.update();
            mechanisms.transferMacro();
            mechanisms.setIntakeSpinners();
            mechanisms.setBaseIntakeLift();
            mechanisms.setBaseOuttakeLift();
            mechanisms.setOutTakeClawGrab();
            mechanisms.setOuttakeArmToNeutralPos();
            mechanisms.setIntakePivot();
            mechanisms.setOuttakePivot();
            //mechanisms.setOutTakeLift();
            //mechanisms.setInTakeLift();
            //mechanisms.setInTakeClawGrab();
            //mechanisms.setInTakeFlip();
            //mechanisms.setOutTakeFlip();
            //mechanisms.setOutTakePivot();
            //mechanisms.setInTakeRotator();
            //mechanisms.outTakeMacroAndTransfer();

            //fileRead.readFile();

            driveTrain.runMotors(sensors);
            mechanisms.changeStaticVals();
            controllerHandler.update();
            //driveTrain.runMotorsConstantSpeed(.3,.3,.3,.3);
            //telemetry.update();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
