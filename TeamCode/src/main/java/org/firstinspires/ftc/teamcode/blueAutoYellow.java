package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

@Autonomous(name = "auto", group = "autos")
public class blueAutoYellow extends LinearOpMode {

    public RRLocalizationRead localizationRead;
    public DcMotor frontL;
    public DcMotor backL;
    public DcMotor frontR;
    public DcMotor backR;

    Servo inTakeClaw;
    Servo inTakeFlipExpansion;
    ElapsedTime totalTime = new ElapsedTime();
    public IMU imu;



@Override
    public void runOpMode() {
    imu = hardwareMap.get(IMU.class, "imu");
    // change it to match the actual orientation of the rev control hub
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    imu.initialize(parameters);
        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap, this);
        backL = hardwareMap.get(DcMotor.class, "backL");
        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        backR = hardwareMap.get(DcMotor.class, "backR");

    frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    backL.setDirection(DcMotorSimple.Direction.REVERSE);

    inTakeFlipExpansion = hardwareMap.servo.get("itfe"); // port 1
    inTakeClaw = hardwareMap.servo.get(("itc")); // port 3

    inTakeClaw.setPosition(1);

        waitForStart();

        if (opModeIsActive()) {
            inTakeFlipExpansion.setPosition(0);
            sleep(1000);
            moveForwardByInches(300.0, 10);  // Moves the robot forward by 10 inches
            inTakeClaw.setPosition(.76);
            sleep(1000);
            moveBackwardByInches(-300.0, 3);
            turnToZero(2);
            moveBackwardByInches(-300, 7);
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    public double returnGyroYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double getAngle()
    {
        return Math.toDegrees(returnGyroYaw());
    }

    void turnToZero(double timeOut)
    {
        timeOut = totalTime.seconds() + timeOut;
        while (opModeIsActive() && Math.abs(getAngle()) > Math.abs(2.5) && totalTime.seconds() <= timeOut)
        {
            backL.setPower(.1);
            frontL.setPower(.1);
            backR.setPower(-.1);
            frontR.setPower(-.1);
        }
    }

    public void moveForwardByInches(double inches, double timeOut) {
        // Record the initial position
        Vector2 initialPosition = localizationRead.returnPose();
        double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis
        timeOut = totalTime.seconds() + timeOut;

        double frontSpeed = .3;

        // Move forward until we reach the target distance
        while (opModeIsActive() && totalTime.seconds() <= timeOut) {
            backL.setPower(frontSpeed);// Apply forward power
            backR.setPower(frontSpeed);
            frontL.setPower(frontSpeed);// Apply forward power
            frontR.setPower(frontSpeed);
            localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        backL.setPower(0);// Apply forward power
        backR.setPower(0); // back right
        frontL.setPower(0);// Apply forward power
        frontR.setPower(0);
    }

    public void moveBackwardByInches(double inches, double timeOut) {
        // Record the initial position

        double backSpeed = -.3;

        Vector2 initialPosition = localizationRead.returnPose();
        double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis
        timeOut = totalTime.seconds() + timeOut;

        // Move forward until we reach the target distance
        while (opModeIsActive() && totalTime.seconds() <= timeOut) {
            backL.setPower(backSpeed);// Apply forward power
            backR.setPower(backSpeed);
            frontL.setPower(backSpeed);// Apply forward power
            frontR.setPower(backSpeed);
            localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        backL.setPower(0);// Apply forward power
        backR.setPower(0);
        frontL.setPower(0);// Apply forward power
        frontR.setPower(0);
    }
}