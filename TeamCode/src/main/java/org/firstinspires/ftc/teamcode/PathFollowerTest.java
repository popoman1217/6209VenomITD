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
@Autonomous(name = "Path Follower Test", group = "autos")
public class PathFollowerTest extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath pathFollower;
    Sensors sensors;
    static int tarPose = 2800;
    public boolean RUNMOTORS = false;
    ControllerHandler controllerHandler = new ControllerHandler();

    @Override
    public void runOpMode() throws InterruptedException{

        RRLocalizationRead rr = new RRLocalizationRead();
        rr.initLocalization(hardwareMap, new Pose2d(0, 0, 0));

        controllerHandler.initController(this);

        pathFollower = new FollowPath();


        pathFollower.Start(this, rr, "/sdcard/FIRST/PathTest.txt");

        ElapsedTime time = new ElapsedTime();

        DrivetrainControllers dt = new DrivetrainControllers();
        dt.initMotorsRR(this, rr);

        sensors = new Sensors();
        double initHeading = 0;
        sensors.init(this, initHeading);

        StateHandler stateHandler = new StateHandler();
        stateHandler.init(this);

        mechanisms = new Mechanisms();
        mechanisms.init(this, dt.frontLeftMotor, dt.backRightMotor, dt.frontRightMotor);
        mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);
        mechanisms.setIntakeZeroPos(0);
        mechanisms.setOutTakeZeroPos(0);
        mechanisms.setIntakeMacroPos(mechanisms.itlPos + 150);

        CommonMechanisms commonMechanisms = new CommonMechanisms();
        commonMechanisms.init(this, stateHandler, mechanisms);

        telemetry.update();

        StateHandler.CoRoutines[] bothCoRoutines = {StateHandler.CoRoutines.ITPID, StateHandler.CoRoutines.OTPID};

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



        stateHandler.switchState("beginning");

        if (stateHandler.state.equals("beginning")) {
            double tarHeading = 5;
            time.reset();
            runPathFollowerMotors(dt, tarHeading, rr);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("move to specimen");
        }

        //commonMechanisms.moveFromClampedToPlaceAndBack("move to specimen");


        if (stateHandler.state.equals("move to specimen")) {
            double tarHeading = 5;
            time.reset();
            pathFollower.incrementTrajNumber();
            runPathFollowerMotors(dt, tarHeading, rr);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("move to place");//"moving intake", 500, mechanisms, StateHandler.CoRoutines.OTPID);
        }


        if (stateHandler.state.equals("moving intake"))
        {
            mechanisms.moveITLiftEncoder(.8, 175, 2);
            mechanisms.powerITPIDToTarget();
            //mechanisms.powerITLift(0);
            stateHandler.stateUpdate();
            stateHandler.switchState("move intake down", 500, mechanisms, bothCoRoutines);
        }

        if (stateHandler.state.equals("move intake down"))
        {
            mechanisms.moveIntakeDown();
            mechanisms.powerSpinners(.7);
            stateHandler.stateUpdate();
            stateHandler.switchState("transfer");
        }

        if (stateHandler.state.equals("transfer"))
        {
            mechanisms.resetMacroVals(true);
            mechanisms.moveITLiftEncoder(.8, 200, 2);
            mechanisms.powerITPIDToTarget();
            mechanisms.transferMacroAuto();
            stateHandler.stateUpdate();
            stateHandler.switchState("move to place", 500, mechanisms, bothCoRoutines);
        }

        if (stateHandler.state.equals("move to place")) {
            double tarHeading = 5;
            time.reset();
            pathFollower.incrementTrajNumber();
            runPathFollowerMotors(dt, tarHeading, rr);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("raising lift", 1000, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        //commonMechanisms.moveFromClampedToPlaceAndBack("none");


        mechanisms.outTakePivotLeft.setPosition(Mechanisms.LOW_OT_ARM_POSL);
        mechanisms.outTakePivotRight.setPosition(Mechanisms.LOW_OT_ARM_POSR);


    }

    public void runPathFollowerMotors(DrivetrainControllers dt, double tarHeading, RRLocalizationRead rr)
    {
        RUNMOTORS = false;
        while (!pathFollower.isAtEnd() && !isStopRequested())
        {
            boolean a = controllerHandler.isGP1APressed1Frame();
            if (a && RUNMOTORS)
                RUNMOTORS = false;
            else if (a)
                RUNMOTORS = true;

            //RUNMOTORS = true;

            pathFollower.update();

            Pose2d pose = rr.returnPose();
            tarHeading = pose.heading.toDouble();
            double adder = sensors.getTrueAngleDiff(tarHeading) * .01;
            //adder = 0;
            telemetry.addData("adder", adder);
            telemetry.addData("heading", sensors.returnGyroYaw());
            telemetry.addLine("x: " + pose.position.x + " y: " + pose.position.y + " heading: " + pose.heading);
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
