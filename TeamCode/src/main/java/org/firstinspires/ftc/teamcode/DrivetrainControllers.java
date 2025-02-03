package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.List;

public class DrivetrainControllers {

    // Field centric stuff
    boolean usingFC = false;
    double initialAngle = 0;
    double prevFCPressTime = 0;

    double curPowerFL = 0;
    double curPowerFR = 0;
    double curPowerBL = 0;
    double curPowerBR = 0;
    //double tarPower = 0;
    double prevTimeFL = 0;
    double prevTimeFR = 0;
    double prevTimeBL = 0;
    double prevTimeBR = 0;

    // Motors for drivetrain
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    ElapsedTime totalTime = new ElapsedTime();

    OpMode master;

    public void init(OpMode opMode)
    {
        frontLeftMotor = opMode.hardwareMap.dcMotor.get("frontL"); // port 0
        backLeftMotor = opMode.hardwareMap.dcMotor.get("backL"); // port 2
        frontRightMotor = opMode.hardwareMap.dcMotor.get("frontR"); // port 1
        backRightMotor = opMode.hardwareMap.dcMotor.get("backR"); // port 3

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        master = opMode;
    }

    public void initMotorsRR(OpMode opMode, RRLocalizationRead rr)
    {
        frontLeftMotor = rr.returnDCMotorBL(MecanumDrive.MotorNames.fl); // port 0
        backLeftMotor = rr.returnDCMotorBL(MecanumDrive.MotorNames.bl); // port 2
        frontRightMotor = rr.returnDCMotorBL(MecanumDrive.MotorNames.fr); // port 1
        backRightMotor = rr.returnDCMotorBL(MecanumDrive.MotorNames.br); // port 3

        master = opMode;
    }

    ////////////////////////////////////////////////////
    double acceleratorFR(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeFR) / 1000.0;
        prevTimeFR = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerFR)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerFR)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerFR += ((Math.pow(curPowerFR - tarPower, 2) * dt) * multiplier);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorFL(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeFL) / 1000.0;
        prevTimeFL = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerFL)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerFL)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerFL += (Math.pow(curPowerFL - tarPower, 2) * dt * multiplier);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorBR(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeBR) / 1000.0;
        prevTimeBR = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerBR)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerBR)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerBR += ((Math.pow(curPowerBR - tarPower, 2) * dt) * multiplier);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorBL(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeBL) / 1000.0;
        prevTimeBL = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerBL)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerBL)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerBL += ((Math.pow(curPowerBL - tarPower, 2) * dt) * multiplier);
        return tarPower;
    }

    ////////////////////////////////////////////////////////////////////////////////
    void runMotors(Sensors sensors)
    {
        // field centric activation
        if (master.gamepad1.b && totalTime.milliseconds() - 500 > prevFCPressTime) {
            if (!usingFC) {
                initialAngle = sensors.returnGyroYaw();
                usingFC = true;
            } else {
                usingFC = false;
            }
            prevFCPressTime = totalTime.milliseconds();
        }

        double adder = sensors.calcAdder();

        if (usingFC) {
            // this would be start on Xbox (changeable)
            if (master.gamepad1.options) {
                sensors.imu.resetYaw();
            }

            double y = -master.gamepad1.left_stick_y;
            double x = master.gamepad1.left_stick_x * 1.1;
            double rx = -master.gamepad1.right_stick_x;

            master.telemetry.addData("y", y);



            double trueDiff = sensors.getTrueAngleDiff(initialAngle);
            double joyAngle = Math.toDegrees(Math.atan2(y, x));
            double trueJoy = 90 - joyAngle;
            if (trueJoy > 180)
            {
                trueJoy = (trueJoy - 360);
            }
            double cos = Math.cos(Math.toRadians(trueJoy - trueDiff));
            double sin = Math.sin(Math.toRadians(trueJoy - trueDiff));

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double multiplier = Math.max(1 - master.gamepad1.right_trigger, .25);

            double power = Math.min(Math.abs(y) + Math.abs(x), 1);
            frontLeftMotor.setPower(acceleratorFL((power * cos + power * sin + rx) * multiplier) - adder);
            backLeftMotor.setPower(-acceleratorBL((power * cos - power * sin + rx) * multiplier) - adder);
            frontRightMotor.setPower(-acceleratorFR((power * cos - power * sin - rx) * multiplier) + adder);
            backRightMotor.setPower(-acceleratorBR((power * cos + power * sin - rx) * multiplier) + adder);
        }
        else {
            double y = -master.gamepad1.left_stick_y;
            double x = master.gamepad1.left_stick_x * 1.1;
            double rx = master.gamepad1.right_stick_x;

            master.telemetry.addData("y", y);
            master.telemetry.addData("x",x);
            master.telemetry.addData("rx",rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double multiplier = Math.max(1 - master.gamepad1.right_trigger, .25) ;

            frontLeftMotor.setPower(-acceleratorFL(frontLeftPower * multiplier) - adder);
            backLeftMotor.setPower(-acceleratorBL(backLeftPower * multiplier) - adder);
            frontRightMotor.setPower(-acceleratorFR(frontRightPower * multiplier) + adder);
            backRightMotor.setPower(-acceleratorBR(backRightPower * multiplier) + adder);
        }
    }

    public void moveForwardByInches(double inches, double timeOut) {
        // Record the initial position
        //Vector2d initialPosition = localizationRead.returnPose().position;
        //double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis
        timeOut = totalTime.seconds() + timeOut;

        // Move forward until we reach the target distance
        while (totalTime.seconds() <= timeOut) {
            frontLeftMotor.setPower(-.3 * Math.signum(inches));// Apply forward power
            frontRightMotor.setPower(.3 * Math.signum(inches));
            backRightMotor.setPower(.3 * Math.signum(inches));// Apply forward power
            backLeftMotor.setPower(.3 * Math.signum(inches));
            //localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        frontLeftMotor.setPower(0);// Apply forward power
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);// Apply forward power
        backRightMotor.setPower(0);
    }


    public void turnPID(double tarHeading, double timeOut, Sensors sensors)
    {
        double kp = 0.03; // Proportional gain
        double ki = 0.0;  // Integral gain
        double kd = 0.0;  // Derivative gain

        double error;
        double previousError = 0;
        double integral = 0;
        double derivative;
        double output;

        timeOut = totalTime.seconds() + timeOut;

        while (totalTime.seconds() <= timeOut) {

            double currentHeading = sensors.returnGyroYaw(); // Assuming you have a method to get the current heading
            error = sensors.getTrueAngleDiff(tarHeading);

            if (Math.abs(error) <= 1)
            {
                break;
            }

            integral += error * (totalTime.milliseconds() - prevTimeFL) / 1000.0;
            derivative = (error - previousError) / ((totalTime.milliseconds() - prevTimeFL) / 1000.0);
            output = kp * error + ki * integral + kd * derivative;

            output = .25 * Math.signum(sensors.getTrueAngleDiff(tarHeading));

            frontLeftMotor.setPower(output);
            frontRightMotor.setPower(-output);
            backLeftMotor.setPower(output);
            backRightMotor.setPower(-output);

            previousError = error;
            prevTimeFL = totalTime.milliseconds();
        }

        // Stop the motors after the turn
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    //////////////////////////////////////////////////////////////////////////////
    public void runTesting()
    {
        if (master.gamepad1.a) {
            frontLeftMotor.setPower(.5);
        }
        else
            frontLeftMotor.setPower(0);
        if (master.gamepad1.b) {
            frontRightMotor.setPower(.5);
        }
        else
            frontRightMotor.setPower(0);
        if (master.gamepad1.x) {
            backLeftMotor.setPower(.5);
        }
        else
            backLeftMotor.setPower(0);
        if (master.gamepad1.y) {
            backRightMotor.setPower(.5);
        }
        else
            backRightMotor.setPower(0);
    }

    ///////////////////////////////////////////////////////////////////////////////
    public void runMotorsConstantSpeed(double powerFL, double powerFR, double powerBL, double powerBR) {
        frontLeftMotor.setPower(powerFL);
        frontRightMotor.setPower(powerFR);
        backLeftMotor.setPower(powerBL);
        backRightMotor.setPower(powerBR);
    }
}
