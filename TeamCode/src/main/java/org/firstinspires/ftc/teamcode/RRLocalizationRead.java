package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    double prevTime = 0;

    // Initial position of x so that you can take that and just subtract what has been changed because it's negative.
    double initialX = 0;

    public void initLocalization(HardwareMap hardwareMap)
    {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        totalTime = new ElapsedTime();
        // Pseudo code for localizer initiation.
        // Should be something like Standardlocalizer localizer = new stsndard(hardwaremap);
    }

    public void initLocalization(HardwareMap hardwareMap, Pose2d pose2d)
    {
        initialX = pose2d.position.x;
        drive = new MecanumDrive(hardwareMap, new Pose2d(pose2d.position.y, pose2d.position.x, pose2d.heading.toDouble()));
        totalTime = new ElapsedTime();
        // Pseudo code for localizer initiation.
        // Should be something like Standardlocalizer localizer = new stsndard(hardwaremap);
    }

    public DcMotor returnDCMotorBL(MecanumDrive.MotorNames name)
    {
        return drive.returnMotor(name);
    }

    public Pose2d returnPose()
    {
        if (drive != null) {
            drive.updatePoseEstimate();
            Pose2d rawPos = new Pose2d(new Vector2d(drive.pose.position.y, drive.pose.position.x), Math.toDegrees(drive.pose.heading.toDouble()));
            // This is to get the direction of change on the x correct without messing with roadrunner's localization.
            return new Pose2d(initialX * 2 - rawPos.position.x, rawPos.position.y, rawPos.heading.toDouble());
        }
        return new Pose2d(new Vector2d(0,0), 0);
        // Pseudo code for localizer.update() and return Pose
    }

    ////////////////////////////////////////////////////////////////////////////////
    public Vector2d returnLinearVel()
    {
        double deltaTime = totalTime.seconds() - prevTime;
        Pose2d thisPos = returnPose();
        Vector2d curV = new Vector2d(Math.sqrt(Math.pow(thisPos.position.x - prevPos.x, 2) + Math.pow(thisPos.position.y - prevPos.y, 2)) / (deltaTime), Math.toDegrees(Math.atan((double)(thisPos.position.y - prevPos.y) / (thisPos.position.x - prevPos.x))));
        // Have to fix for it to be relative, I think it already is though
        return curV;
    }


}
