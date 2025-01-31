package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name = "Path Follower Test", group = "autos")
public class PathFollowerTest extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath pathFollower;
    Sensors sensors;
    static int tarPose = 2500;
    public boolean RUNMOTORS = false;
    ControllerHandler controllerHandler = new ControllerHandler();

    @Override
    public void runOpMode() throws InterruptedException{

        RRLocalizationRead rr = new RRLocalizationRead();

        controllerHandler.initController(this);

        pathFollower = new FollowPath();

        rr.initLocalization(hardwareMap);

        pathFollower.Start(this, rr);

        ElapsedTime time = new ElapsedTime();

        DrivetrainControllers dt = new DrivetrainControllers();
        dt.initMotorsRR(this, rr);

        sensors = new Sensors();
        sensors.init(this);

        telemetry.update();

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



        double tarHeading = 0;
        time.reset();
        while (!pathFollower.isAtEnd() && !isStopRequested())
        {
            boolean a = controllerHandler.isGP1APressed1Frame();
            if (a && RUNMOTORS)
                RUNMOTORS = false;
            else if (a)
                RUNMOTORS = true;

            pathFollower.update();

            Pose2d pose = rr.returnPose();
            tarHeading = pose.heading.toDouble();
            double adder = sensors.getTrueAngleDiff(tarHeading) * .07;
            adder = 0;
            telemetry.addData("adder", adder);
            telemetry.addData("heading", sensors.returnGyroYaw());
            telemetry.addLine("x: " + pose.position.x + " y: " + pose.position.y + " heading: " + pose.heading);
            telemetry.addData("RUNMOTORS", RUNMOTORS);
            double[] traj = pathFollower.getRobotTrajectory();
            telemetry.addLine("dirx " + traj[0] + "dirY " + traj[1]);
            double y = traj[1];
            double x = traj[0];
            if (RUNMOTORS) {
                double frontLeftPower = (y + x - adder);
                double backLeftPower = (y - x - adder);
                double frontRightPower = (y - x + adder);
                double backRightPower = (y + x + adder);

                telemetry.addData("fl power", frontLeftPower);

                dt.frontRightMotor.setPower(frontRightPower * .5);
                dt.frontLeftMotor.setPower(frontLeftPower * .5);
                dt.backRightMotor.setPower(backRightPower * .5);
                dt.backLeftMotor.setPower(backLeftPower * .5);

                telemetry.addData("fl motor", dt.frontLeftMotor.getPortNumber());
            }
            else
            {
                dt.frontRightMotor.setPower(0);
                dt.frontLeftMotor.setPower(0);
                dt.backRightMotor.setPower(0);
                dt.backLeftMotor.setPower(0);
            }

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.update();

            telemetry.update();
        }
        dt.frontRightMotor.setPower(0);
        dt.frontLeftMotor.setPower(0);
        dt.backRightMotor.setPower(0);
        dt.backLeftMotor.setPower(0);

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
