package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{


        controllerHandler.initController(this);

        pathFollower = new FollowPath();



        sensors = new Sensors();
        double initHeading = 0;
        sensors.init(this, initHeading);

        pathFollower.Start(this, "/sdcard/FIRST/PathTest.txt");

        ElapsedTime time = new ElapsedTime();

        DrivetrainControllers dt = new DrivetrainControllers();
        dt.initMotorsRR(this, pathFollower.posReader);

        StateHandler stateHandler = new StateHandler();
        stateHandler.init(this);

        mechanisms = new Mechanisms();
        mechanisms.init(this, dt.frontLeftMotor, dt.backRightMotor, dt.frontRightMotor);
        mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);
        mechanisms.setIntakeZeroPos(0);
        mechanisms.setOutTakeZeroPos(0);
        mechanisms.setIntakeMacroPos(mechanisms.itlPos + 225);

        CommonMechanisms commonMechanisms = new CommonMechanisms();
        commonMechanisms.init(this, stateHandler, mechanisms);


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

        pathFollower.initPostInitialization(sensors);
        Vector2 fieldPos = pathFollower.getFieldPos();

        telemetry.addData("x", fieldPos.x);
        telemetry.addData("y", fieldPos.y);

        telemetry.update();
        //mechanisms.moveIntakeDown();

        waitForStart();
        if(isStopRequested()) return;

        time.reset();
        while (time.milliseconds() < 200) {
            pathFollower.initPostStart();
            telemetry.update();
        }
        //pathFollower.posReader.initLocalization(hardwareMap, new Pose2d(fieldPos.x, fieldPos.y, sensors.returnGyroYaw()));



       /* while (!isStopRequested()) {
            Vector2 pos = pathFollower.posReader.returnPose();
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("Pose heading", pathFollower.posReader.returnHeading());
            telemetry.addData("heading", sensors.returnGyroYaw());
            telemetry.update();
        }*/
        //pathFollower.posReader.returnPose();
        //pathFollower.posReader.resetPos(new Pose2d(fieldPos.x, fieldPos.y, sensors.returnGyroYaw()));

        stateHandler.switchState("beginning");

        if (stateHandler.state.equals("beginning")) {
            double tarHeading = -5;
            time.reset();
            pathFollower.runPathFollowerMotors(dt, tarHeading, pathFollower.posReader, controllerHandler, this);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("place", 500, mechanisms, StateHandler.CoRoutines.None);
        }

        commonMechanisms.moveFromClampedToPlaceAndBack("moving intake");

        if (stateHandler.state.equals("moving intake"))
        {
            mechanisms.moveITLiftEncoder(.8, 250, 2);
            mechanisms.powerITPIDToTarget();
            //mechanisms.powerITLift(0);
            stateHandler.stateUpdate();
            stateHandler.switchState("move intake down", 500, mechanisms, bothCoRoutines);
        }

        if (stateHandler.state.equals("move intake down"))
        {
            mechanisms.moveIntakeDown();
            mechanisms.powerSpinners(1);
            stateHandler.stateUpdate();
            stateHandler.switchState("move intake further");
        }

        if (stateHandler.state.equals("move intake further"))
        {
            mechanisms.resetMacroVals(true);
            mechanisms.moveITLiftEncoder(.8, 200, 2);
            mechanisms.powerITPIDToTarget();
            stateHandler.stateUpdate();
            stateHandler.switchState("move to specimen", 500, mechanisms, bothCoRoutines);
        }


        if (stateHandler.state.equals("move to specimen")) {
            double tarHeading = -3;
            time.reset();
            pathFollower.incrementTrajNumber();
            pathFollower.runPathFollowerMotors(dt, tarHeading, pathFollower.posReader, controllerHandler, this);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            dt.oscillateMotors(.3,.7,sensors, pathFollower.posReader);
            stateHandler.switchState("transfer", 300, mechanisms, bothCoRoutines);//"moving intake", 500, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        if (stateHandler.state.equals("transfer")) {
            mechanisms.transferMacroAuto();
            stateHandler.stateUpdate();
            stateHandler.switchState("move to place", 100, mechanisms, bothCoRoutines);
        }

        if (stateHandler.state.equals("move to place")) {
            double tarHeading = -5;
            time.reset();
            pathFollower.incrementTrajNumber();
            pathFollower.runPathFollowerMotors(dt, tarHeading, 3, pathFollower.posReader, controllerHandler, this);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("raising lift", 1000, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        commonMechanisms.moveFromClampedToPlaceAndBack("moveToPark");

        mechanisms.moveIntakeNeutral();


        mechanisms.outTakePivotLeft.setPosition(Mechanisms.LOW_OT_ARM_POSL);
        mechanisms.outTakePivotRight.setPosition(Mechanisms.LOW_OT_ARM_POSR);

        if (stateHandler.state.equals("move to place")) {
            double tarHeading = 0;
            time.reset();
            pathFollower.incrementTrajNumber();
            pathFollower.runPathFollowerMotors(dt, tarHeading, 6, pathFollower.posReader, controllerHandler, this);
            dt.frontRightMotor.setPower(0);
            dt.frontLeftMotor.setPower(0);
            dt.backRightMotor.setPower(0);
            dt.backLeftMotor.setPower(0);
            stateHandler.switchState("none");
        }


    }


}
