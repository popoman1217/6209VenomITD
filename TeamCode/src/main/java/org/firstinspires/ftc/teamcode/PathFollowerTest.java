package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name = "Path Follower Test", group = "autos")
public class PathFollowerTest extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath pathFollower;
    static int tarPose = 2500;
    @Override
    public void runOpMode() throws InterruptedException{

        RRLocalizationRead rr = new RRLocalizationRead();

        pathFollower = new FollowPath();

        rr.initLocalization(hardwareMap);

        pathFollower.Start(this, rr);

        ElapsedTime time = new ElapsedTime();

        DrivetrainControllers dt = new DrivetrainControllers();
        dt.initMotorsRR(this, rr);

        //RRLocalizationRead rrLocalizationRead = new RRLocalizationRead();
        //rrLocalizationRead.initLocalization(hardwareMap);

        //drivetrainControllers.frontLeftMotor

        //DrivetrainControllers dt = new DrivetrainControllers();
        //dt.init(this);

        //followPath = new FollowPath();
        //followPath.Start(this, rrLocalizationRead);
        //    .setTangent(Math.PI/2)

        //mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);

        waitForStart();
        if(isStopRequested()) return;



        time.reset();
        while (time.milliseconds() < 10000 && !pathFollower.isAtEnd())
        {
            pathFollower.update();
            double[] traj = pathFollower.getRobotTrajectory();
            telemetry.addLine("dirx " + traj[0] + "dirY " + traj[1]);
            double y = -traj[1];
            double x = traj[0];
            double frontLeftPower = (y + x);
            double backLeftPower = (y - x);
            double frontRightPower = (y - x);
            double backRightPower = (y + x);

            dt.frontRightMotor.setPower(frontRightPower * .3);
            dt.frontLeftMotor.setPower(frontLeftPower * .3);
            dt.backRightMotor.setPower(backRightPower * .3);
            dt.backLeftMotor.setPower(backLeftPower * .3);

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.update();

            telemetry.update();
        }

            /*
            mecanumDrive.
            telemetry.addData("x", rrLocalizationRead.returnPose().position.x);
            telemetry.addData("y", rrLocalizationRead.returnPose().position.y);
            telemetry.update();


            followPath.update();
            double[] traj = followPath.getRobotTrajectory();
            double y = -traj[1];
            double x = traj[0];
            double frontLeftPower = (y + x);
            double backLeftPower = (y - x);
            double frontRightPower = (y - x);
            double backRightPower = (y + x);

            dt.frontRightMotor.setPower(frontRightPower * .3);
            dt.frontLeftMotor.setPower(frontLeftPower * .3);
            dt.backRightMotor.setPower(backRightPower * .3);
            dt.backLeftMotor.setPower(backLeftPower * .3);

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.update();



            //driveTrain.moveForwardByInches(-60, 10);// Moves the robot forward by 10 inches

        }*/
    }
}
