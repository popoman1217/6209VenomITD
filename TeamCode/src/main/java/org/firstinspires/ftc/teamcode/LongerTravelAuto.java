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

@Autonomous(name = "longer travel auto", group = "autos")
public class LongerTravelAuto extends LinearOpMode {
    Mechanisms mechanisms;
    FollowPath followPath;
    static int tarPose = 2500;
    @Override
    public void runOpMode() throws InterruptedException{

        mechanisms = new Mechanisms();

        ElapsedTime time = new ElapsedTime();

        //RRLocalizationRead rrLocalizationRead = new RRLocalizationRead();
        //rrLocalizationRead.initLocalization(hardwareMap);

        //drivetrainControllers.frontLeftMotor


        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mechanisms.init(this, mecanumDrive.returnMotor(MecanumDrive.MotorNames.fl), mecanumDrive.returnMotor(MecanumDrive.MotorNames.br), mecanumDrive.returnMotor(MecanumDrive.MotorNames.fr));

        //DrivetrainControllers dt = new DrivetrainControllers();
        //dt.init(this);

        //followPath = new FollowPath();
        //followPath.Start(this, rrLocalizationRead);
        TrajectoryActionBuilder forward = mecanumDrive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(-9);
        //    .setTangent(Math.PI/2)

        mechanisms.outTakeClaw.setPosition(mechanisms.GRAB_CLAW_POS);

        waitForStart();
        if(isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        forward.build()
                )
        );
        mechanisms.moveOTLiftEncoder(.7, tarPose, 7000);
        mechanisms.setMacroBrakeValsOT();
        time.reset();
        while (time.milliseconds() < 2000) {
            mechanisms.outTakePivotLeft.setPosition(.65);
            mechanisms.outTakePivotRight.setPosition(mechanisms.HIGH_OT_ARM_POSR);
            if (time.milliseconds() > 500)
                mechanisms.outTakeClaw.setPosition(mechanisms.OPEN_CLAW_POS);
            mechanisms.setOTBrake();
        }
        mechanisms.outTakePivotLeft.setPosition(.55);
        mechanisms.outTakePivotRight.setPosition(mechanisms.HIGH_OT_ARM_POSR - .1);

        while (time.milliseconds() < 3000)
        {
            mechanisms.setMacroBrakeValsOT();
        }
        mechanisms.moveOTLiftEncoder(.7, -1000, 1000);

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
