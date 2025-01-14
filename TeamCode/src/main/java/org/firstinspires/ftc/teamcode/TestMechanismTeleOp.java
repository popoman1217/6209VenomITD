package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MechanismTestTeleOp", group = "Teleops")
@Config
public class TestMechanismTeleOp extends OpMode{

    public static boolean[] motorPorts = new boolean[8];
    public static boolean[] servoPorts = new boolean[12];

    ElapsedTime stateTime = new ElapsedTime();


    int curMotor = 0;
    int curServo = 0;

    double curServoPos = 0;
    double curMotorPos = 0;

    ElapsedTime clickTime = new ElapsedTime();
    String curState = "general testing";

    DcMotor[] motors = new DcMotor[8];
    Servo[] servos = new Servo[12];

    @Override
    public void init() {
        checkInit();
        int index = 0;
        for (DcMotor motor : motors)
        {
            //if (motor != null)
            {
                curMotor = index;
            }
            index++;
        }
        index = 0;
        for (Servo servo : servos)
        {
            //if (servo != null)
            {
                curServo = index;
            }
            index++;
        }
        telemetry.update();
    }

    void checkInit()
    {
        int index = 0;
        for (boolean port : motorPorts)
        {
            telemetry.addData("in", port);
            motors[index] = hardwareMap.dcMotor.get("motor" + index);
            telemetry.addData("motor", motors[index]);
            index++;
        }

        index = 0;
        for (boolean port : servoPorts)
        {
            telemetry.addData("in", port);
            servos[index] = hardwareMap.servo.get("servo" + index);
            telemetry.addData("servo", servos[index]);
            return;
        }
    }

    @Override
    public void loop() {
        if (gamepad2.a)
        {
            curState = "intake testing";
            switchState();
        }
        else if (gamepad2.b)
        {
            curState = "general testing";
            switchState();
        }

        if (curState.equals("general testing"))
            testMotorAndServo();

        if (curState.equals("intake testing"))
            inTakeMacroTest(motors[curMotor], 1000);

        telemetry.update();
    }

    public void testMotorAndServo()
    {
        if (gamepad1.dpad_right && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            for (int i = curMotor + 1; i < motors.length + curMotor; i++)
            {
                if (motors[i % motors.length] != null) {
                    curMotor = i % motors.length;
                    break;
                }
            }
        }
        else if (gamepad1.dpad_left && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            for (int i = curMotor - 1; i > curMotor - motors.length; i--)
            {
                if (motors[i % motors.length] != null) {
                    curMotor = i % motors.length;
                    break;
                }
            }
        }
        else if (gamepad1.dpad_up && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            for (int i = curServo + 1; i < servos.length + curServo; i++)
            {
                if (servos[i % servos.length] != null) {
                    curServo = i % servos.length;
                    break;
                }
            }
        }
        else if (gamepad1.dpad_down && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            for (int i = curServo - 1; i > curServo - servos.length; i--)
            {
                if (servos[i % servos.length] != null) {
                    curServo = i % servos.length;
                    break;
                }
            }
        }

        if (motors[curMotor] != null)
        {
            motors[curMotor].setPower(gamepad1.left_stick_y);
            telemetry.addData("CurMotor", curMotor);
            telemetry.addData("motor pos", motors[curMotor].getCurrentPosition());
        }
        if (servos[curServo] != null)
        {
            if (gamepad1.right_bumper && clickTime.milliseconds() > 500)
            {
                curServoPos += .1;
                servos[curServo].setPosition(curServoPos);
                clickTime.reset();
            }
            else if (gamepad1.left_bumper && clickTime.milliseconds() > 500)
            {
                curServoPos -= .1;
                servos[curServo].setPosition(curServoPos);
                clickTime.reset();
            }
            telemetry.addData("CurServo", curServo);
            telemetry.addData("servo pos", curServoPos);
        }
    }

    public void inTakeMacroTest(DcMotor intakeLiftMotor, int pos)
    {
        // intakeLiftMotor is just the motor in the array which corresponds to the intake lift at the moment.
        intakeLiftMotor.setPower((pos - motors[5].getCurrentPosition()) / 100.0 * .07);
    }

    public void outTakeMacroTest(DcMotor outtakeLiftMotor, int pos)
    {
        // intakeLiftMotor is just the motor in the array which corresponds to the intake lift at the moment.
        outtakeLiftMotor.setPower((pos - outtakeLiftMotor.getCurrentPosition()) / 100.0 * .07);
    }

    public void switchState()
    {
        stateTime.reset();
        for (DcMotor motor : motors)
        {
            if (motor != null)
            {
                motor.setPower(0);
            }
        }
    }
}
