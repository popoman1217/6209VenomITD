package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class RRLocalizationRead {
    MecanumDrive drive;
    ElapsedTime totalTime;
    Vector2d prevPos = new Vector2d(0,0);
    double prevVel = 0;
    double curVel = 0;
    Sensors sensors;
    Vector2 xPrime;
    Vector2 yPrime;
    Vector2 initPos;
    double initHeading = 0;
    Vector2 initShifted = new Vector2(0,0);
    OpMode master;

    // Initial position of x so that you can take that and just subtract what has been changed because it's negative.
    double initialX = 0;
    HardwareMap hardwareMap;
    boolean fullyInitialized = false;
    double dt = 0;
    double prevTime = 0;

    // This class is solely meant to get the initial position and user directed heading offset of the robot, during initialization and
    // "post-initialization" initialization, respectively, and then the robot's position and velocity during the match.
    // "Post-start" initialization is meant to get the shift of the robot during initialization period.

    // The initLocalizations are meant solely for creating the localizer object and setting the initial POSITION of the robot NOT the heading.
    public void initLocalization(HardwareMap hMap, OpMode opMode)
    {
        hardwareMap = hMap;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        initPos = new Vector2(0,0);
        initHeading = 0;
        totalTime = new ElapsedTime();
        // Pseudo code for localizer initiation.
        // Should be something like Standardlocalizer localizer = new stsndard(hardwaremap);
    }

    public void initLocalization(HardwareMap hardwareMap, Vector2 pose2d, OpMode opMode)
    {
        master = opMode;
        initialX = pose2d.x;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        initPos = new Vector2(pose2d.x, pose2d.y);
        totalTime = new ElapsedTime();
        // Pseudo code for localizer initiation.
        // Should be something like Standardlocalizer localizer = new stsndard(hardwaremap);
    }

    // This is solely for the purpose of getting the user-initiated heading offset of the robot at the start of the match.
    // i.e. if the robot is facing 90 degrees, the user can set the heading to 90 degrees and it will be added to the gyro reading.
    // Positive is clockwise and negative is counterclockwise.
    public void initPostInitialization(Sensors s)
    {
        sensors = s;
        if (sensors != null)
            initHeading = sensors.returnGyroYaw();
        else
            initHeading = 0;

        double yaw = initHeading;
        xPrime = new Vector2(Math.cos(Math.toRadians(yaw)), Math.sin(Math.toRadians(yaw)));
        yPrime = new Vector2(Math.cos(Math.toRadians(yaw + 90)), Math.sin(Math.toRadians(yaw + 90)));
    }

    // This method is for setting the initial shift of the robot as it moved during initialization as well as
    // Setting the xPrime and yPrime based off of the initial offset of the heading of the robot directed by the user.
    public void initPostStart()
    {
        double yaw = returnHeading() - initHeading;
        xPrime = new Vector2(Math.cos(Math.toRadians(yaw)), -Math.sin(Math.toRadians(yaw)));
        yPrime = new Vector2(-Math.cos(Math.toRadians(yaw + 90)), Math.sin(Math.toRadians(yaw + 90)));
        // For getting the initial dt.
        prevTime = totalTime.milliseconds();
        dt = Math.max(totalTime.milliseconds() - prevTime, .00001);
        drive.updatePoseEstimate();
        initShifted = new Vector2(0, 0);
        Vector2 curInitShifted = new Vector2(drive.pose.position.y, drive.pose.position.x);

        // Repeatedly updates the pose estimate and gets the change in x and y positions per change in time and until those reach nearly zero
        // It keeps repeating.
        while (Math.abs(initShifted.x - curInitShifted.x) / dt > .0001  && Math.abs(initShifted.y - curInitShifted.y) / dt > .0001)
        {
            initShifted = new Vector2(drive.pose.position.y, drive.pose.position.x);
            drive.updatePoseEstimate();
            curInitShifted = new Vector2(drive.pose.position.y, drive.pose.position.x);
            master.telemetry.addData("pos y", drive.pose.position.x);
            master.telemetry.addData("pos x", drive.pose.position.y);
            dt = Math.max(totalTime.milliseconds() - prevTime, .00001);
            prevTime = totalTime.milliseconds();
        }

        // Sets the final initial shift to the position of the robot.
        initShifted = new Vector2(drive.pose.position.y, drive.pose.position.x);
        master.telemetry.addData("pos y", drive.pose.position.x);
        master.telemetry.addData("pos x", drive.pose.position.y);
        fullyInitialized = true;
        prevTime = totalTime.milliseconds();
    }


    public void resetPos(){
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    public void resetPos(Pose2d pose2d){
        drive.setPoseEstimate(new Pose2d(pose2d.position.y, pose2d.position.x, Math.toRadians(pose2d.heading.toDouble())));
    }

    public DcMotor returnDCMotorBL(MecanumDrive.MotorNames name)
    {
        return drive.returnMotor(name);
    }

    public Vector2 returnPose()
    {
        if (drive != null) {
            if (fullyInitialized) {
                drive.updatePoseEstimate();
                double rot = drive.pose.heading.toDouble();
                Pose2d rawPos = new Pose2d(new Vector2d(drive.pose.position.y, drive.pose.position.x), rot);
                // This is to get the direction of change on the x correct without messing with roadrunner's localization.
                master.telemetry.addData("pos raw x", drive.pose.position.x);
                master.telemetry.addData("pos raw y", drive.pose.position.y);
                double translatedX = (rawPos.position.x - initShifted.x) * xPrime.x + (rawPos.position.y - initShifted.y) * yPrime.x;
                double translatedY = (rawPos.position.x - initShifted.x) * xPrime.y + (rawPos.position.y - initShifted.y) * yPrime.y;
                master.telemetry.addData("init shifted x", initShifted.x);
                master.telemetry.addData("init shifted y", initShifted.y);
                double shiftedX = initPos.x - translatedX;
                double shiftedY = initPos.y + translatedY;
                return new Vector2(shiftedX, shiftedY);
            }
            else
                return new Vector2(initPos.x, initPos.y);
        }
        return new Vector2(0,0);
        // Pseudo code for localizer.update() and return Pose
    }

    public double returnHeading()
    {
        if (sensors != null)
            return sensors.returnGyroYaw();
        else
            return 0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    public Vector2d returnLinearVel()
    {
        double deltaTime = totalTime.seconds() - prevTime;
        Vector2 thisPos = returnPose();
        Vector2d curV = new Vector2d(Math.sqrt(Math.pow(thisPos.x - prevPos.x, 2) + Math.pow(thisPos.y - prevPos.y, 2)) / (deltaTime), Math.toDegrees(Math.atan((double)(returnHeading() - prevPos.y) / (thisPos.x - prevPos.x))));
        // Have to fix for it to be relative, I think it already is though
        return curV;
    }


}
