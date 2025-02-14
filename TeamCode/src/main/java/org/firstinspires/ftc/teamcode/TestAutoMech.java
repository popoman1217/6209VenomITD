package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "TestAutoMech", group = "autos")
public class TestAutoMech extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath pathFollower;
    Sensors sensors;
    static int tarPose = 2100;
    public boolean RUNMOTORS = false;
    ControllerHandler controllerHandler = new ControllerHandler();
    CommonMechanisms commonMechanisms;
    StateHandler stateHandler;
    String state = "beginning";
    String prevState = "none";
    ElapsedTime stateTime = new ElapsedTime();
    boolean firstStateFrame = true;


    public void runOpMode()
    {

        stateHandler = new StateHandler();
        stateHandler.init(this);

        RRLocalizationRead rr = new RRLocalizationRead();

        controllerHandler.initController(this);

        pathFollower = new FollowPath();

        commonMechanisms = new CommonMechanisms();

        rr.initLocalization(hardwareMap, new Vector2(0, 0), this);

        pathFollower.Start(this, "/sdcard/FIRST/PathTest.txt");

    ElapsedTime time = new ElapsedTime();

    DrivetrainControllers dt = new DrivetrainControllers();
    dt.initMotorsRR(this, rr);

    sensors = new Sensors();
    double initHeading = 0;
        sensors.init(this, initHeading);


    mechanisms = new Mechanisms();
        mechanisms.init(this, dt.frontLeftMotor, dt.backRightMotor, dt.frontRightMotor);
        mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);
        mechanisms.setIntakeZeroPos(0);
        mechanisms.setOutTakeZeroPos(0);
        mechanisms.setIntakeMacroPos(mechanisms.itlPos + 150);

        telemetry.update();
    commonMechanisms.init(this, stateHandler, mechanisms);


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

        switchState("beginning");

        commonMechanisms.moveFromClampedToPlaceAndBack("moving intake");

        if (state.equals("moving intake"))
        {
            mechanisms.moveITLiftEncoder(.8, 175, 4);
            mechanisms.powerITPIDToTarget();
            //mechanisms.powerITLift(0);
            stateUpdate();
            switchState("move intake down", 1000);
        }

        if (state.equals("move intake down"))
        {
            mechanisms.moveIntakeDown();
            mechanisms.powerSpinners(.7);
            stateUpdate();
            switchState("transfer");
        }

        if (state.equals("transfer"))
        {
            mechanisms.resetMacroVals(true);
            mechanisms.moveITLiftEncoder(.8, 200, 4);
            mechanisms.powerITPIDToTarget();
            mechanisms.transferMacroAuto();
            stateUpdate();
            switchState("raising lift");
        }

        commonMechanisms.moveFromClampedToPlaceAndBack("none");


    }


    private void switchState(String newState)
    {
        telemetry.addLine("switching state");
        telemetry.addData("old state", state);
        prevState = state;
        state = newState;
        stateTime.reset();
        firstStateFrame = true;
        telemetry.addData("new state", state);
        telemetry.update();
    }

    private void switchState(String newState, long delay)
    {
        ElapsedTime delayTime = new ElapsedTime();
        telemetry.addLine("switching state");
        telemetry.addData("old state", state);
        prevState = state;
        state = newState;
        stateTime.reset();
        firstStateFrame = true;
        telemetry.addData("new state", state);
        telemetry.addData("sleeping X seconds", delay);
        while (delayTime.milliseconds() < delay)
        {
            mechanisms.update();
            mechanisms.powerOTPIDToTarget();
            mechanisms.powerITPIDToTarget();
        }
        telemetry.update();
    }

    private void stateUpdate()
    {
        telemetry.addData("old state", prevState);
        telemetry.addData("new state", state);
        telemetry.addData("state time", stateTime.milliseconds());
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

        RUNMOTORS = true;

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
