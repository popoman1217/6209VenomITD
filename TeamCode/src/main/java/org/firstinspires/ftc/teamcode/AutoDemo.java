package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "AutoDemo", group = "autos")
public class AutoDemo extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath pathFollower;
    Sensors sensors;
    static int tarPose = 800;
    public boolean RUNMOTORS = false;
    ControllerHandler controllerHandler = new ControllerHandler();

    @Override
    public void runOpMode() throws InterruptedException{

        RRLocalizationRead rr = new RRLocalizationRead();

        controllerHandler.initController(this);

        pathFollower = new FollowPath();

        //rr.initLocalization(hardwareMap, new Pose2d(0, 0, 0));

        pathFollower.Start(this, "/sdcard/FIRST/AutoDemo.txt");

        ElapsedTime time = new ElapsedTime();

        DrivetrainControllers dt = new DrivetrainControllers();
        dt.initMotorsRR(this, pathFollower.posReader);

        sensors = new Sensors();
        double initHeading = 0;
        sensors.init(this, initHeading);


        mechanisms = new Mechanisms();
        mechanisms.init(this, dt.frontLeftMotor, dt.backRightMotor, dt.frontRightMotor);
        mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);
        mechanisms.setIntakeZeroPos(0);

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
        runPathFollowerMotors(dt, tarHeading, rr);

    }

    public void runPathFollowerMotors(DrivetrainControllers dt, double tarHeading, RRLocalizationRead rr)
    {
        while (!pathFollower.isAtEnd() && !isStopRequested())
        {
            boolean a = controllerHandler.isGP1APressed1Frame();
            if (a && RUNMOTORS)
                RUNMOTORS = false;
            else if (a)
                RUNMOTORS = true;

            // RUNMOTORS = true;

            pathFollower.update();

            Vector2 pose = rr.returnPose();
            double adder = sensors.getTrueAngleDiff(tarHeading) * .01;
            //adder = 0;
            telemetry.addData("adder", adder);
            telemetry.addData("heading", sensors.returnGyroYaw());
            telemetry.addLine("x: " + pose.x + " y: " + pose.y + " heading: " + rr.returnHeading());
            telemetry.addData("RUNMOTORS", RUNMOTORS);
            double[] traj = pathFollower.getRobotTrajectory();
            telemetry.addLine("dirx " + traj[0] + "dirY " + traj[1]);
            double y = traj[1];
            double x = traj[0];
            double heading = sensors.returnGyroYaw();
            double multiplier = pathFollower.getSpeedController();
            double trueX = x * Math.cos(Math.toRadians(heading)) - y * Math.sin(Math.toRadians(heading));
            double trueY = x * Math.sin(Math.toRadians(heading)) + y * Math.cos(Math.toRadians(heading));
            if (RUNMOTORS) {
                double frontLeftPower = (trueY + trueX - adder) * multiplier;
                double backLeftPower = (trueY - trueX - adder) * multiplier;
                double frontRightPower = (trueY - trueX + adder) * multiplier;
                double backRightPower = (trueY + trueX + adder) * multiplier;

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
    }
}
