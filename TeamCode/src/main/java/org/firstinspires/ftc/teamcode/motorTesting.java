package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// This is the class for all the calling and organizing of the functions. It runs the logic on the WHOLE
// teleop, not the specific "buttons doing what" stuff, but deciding what should and shouldn't run
// based off of input from the specific classes.
@TeleOp(name = "motorTesting", group = "Teleops")
@Config
public class motorTesting extends OpMode {

    DcMotor curMotor;


    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {

        curMotor = hardwareMap.dcMotor.get("cm");
        curMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {
        if (gamepad2.right_trigger > 0.4)
        {
            curMotor.setPower(1);
        }
        else if (gamepad2.left_trigger == 0)
        {
            curMotor.setPower(0);
        }
        else if (gamepad2.left_trigger > 0.4)
        {
            curMotor.setPower(-1);
        }

    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
