package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
public class Mechanisms {

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime outtakeTransferPos = new ElapsedTime();
    ElapsedTime inTakeStateTime = new ElapsedTime();

    // outake lift right
    DcMotor outTakeLiftRight;
    // outake lift left
    DcMotor outTakeLiftLeft;

    // outtake servos
    Servo outTakePivotRight;
    Servo outTakePivotLeft;
    Servo outTakeClaw;
    //Servo outTakeFlip;
    Servo hyperServo;

    // intake lifts
    DcMotor inTakeLift;
    DcMotor inTakeSpinners;

    // intake servos
    Servo intakePivotR;
    Servo intakePivotL;

    // color sensor in intake
    ColorSensor intakeColorSensor;

    // color sensor values
    double redValue;
    double blueValue;
    double greenValue;
    double alphaValue; // light intensity
    double tarVal = 1000;

    int leftOTLPos = 0;
    int rightOTLPos = 0;

    double kpOT = 0;
    double kpIT = 0;

    boolean isStraight = true;

    // Mechanism stuff
    public double lastClawTime;

    int targetOTLPosR = 0;
    int targetOTLPosL = 0;
    boolean approachingTarOT = false;

    int targetITLiftPos = 0;
    boolean approachingTarIT = false;

    String inTakeState = "up";

    boolean transferringOTPivot;
    ElapsedTime otPivotTime = new ElapsedTime();

    boolean TESTINGOTARML = false;

    int itlPos = 0;
    /*public static double PLACE_OT_FLIP_POS = .35;
    public static double UP_OT_FLIP_POS = .25;
    public static double MID_OT_FLIP_POS = .15;
    public static double DOWN_OT_FLIP_POS = 0;
    public static double BUCKET_OT_PIVOT_POS = .5;
    public static double TRANSFER_OT_PIVOT_POS = .67;
    public static double MIDDLE_OT_PIV_POS = .5; // still needs to be tuned
    public static double PICKUP_OT_PIVOT_POS = .23;

    public static double OT_CLAW_GRAB = .68;
    public static double OT_CLAW_RELEASE = 1;

    public static double PERP_IT_POS = .82;
    public static double PAR_IT_POS = .33;
    public static double CLOSED_IT_POS = 1;
    public static double OPEN_IT_POS = .76;
    public static double MIDDLE_IT_POS = .88;
    public static double UP_IT_FLIP_POS = 1;
    public static double DOWN_IT_FLIP_POS = .29;
    public static double MID_IT_FLIP_POS = .5;*/






    // done
    public static double HIGH_OT_ARM_POSL = 0.6;
    public static double LOW_OT_ARM_POSL = 0.19;
    public static double NEUTRAL_OT_ARM_POSL = 0.55;

    public static double HIGH_OT_ARM_POSR = 0.03;
    public static double LOW_OT_ARM_POSR = .44;
    public static double NEUTRAL_OT_ARM_POSR = .08;

    public static double STRAIGHT_OT_FLIP_POS = 0.6;
    public static double BENT_OT_FLIP_POS = .3;
    //public static double NEUTRAL_OT_FLIP_POS = .8;



    // done
    public static double GRAB_CLAW_POS = 0.06;
    public static double OPEN_CLAW_POS = 0.3;
    public static double NEUTRAL_CLAW_POS = 0.2;

    public static double LOW_IT_FLIP_POSR = 0.07;
    public static double LOW_IT_FLIP_POSL = 0.94;
    public static double NEUTRAL_IT_FLIP_POSR = 0.38;
    public static double NEUTRAL_IT_FLIP_POSL = 0.6;
    public static double HIGH_IT_FLIP_POSR = 0.66;
    public static double HIGH_IT_FLIP_POSL = 0.38;
    public static double EASE_IT_FLIP_POSR = 0.1;
    public static double EASE_IT_FLIP_POSL = 0.91;

    public double HIGH_IT_FLIP_POSL_CHANGE = 0;
    public double HIGH_IT_FLIP_POSR_CHANGE = 0;
    public double EASE_IT_FLIP_POSL_CHANGE = 0;
    public double EASE_IT_FLIP_POSR_CHANGE = 0;
    public double LOW_IT_FLIP_POSL_CHANGE = 0;
    public double LOW_IT_FLIP_POSR_CHANGE = 0;
    public double NEUTRAL_IT_FLIP_POSL_CHANGE = 0;
    public double NEUTRAL_IT_FLIP_POSR_CHANGE = 0;

    public double HIGH_OT_ARM_POSL_CHANGE = 0;
    public double HIGH_OT_ARM_POSR_CHANGE = 0;
    public double LOW_OT_ARM_POSL_CHANGE = 0;
    public double LOW_OT_ARM_POSR_CHANGE = 0;
    public double NEUTRAL_OT_ARM_POSL_CHANGE = 0;
    public double NEUTRAL_OT_ARM_POSR_CHANGE = 0;

    public staticVars prevValChanged = staticVars.NONE;


    ElapsedTime intakeToTransfer = new ElapsedTime();

    String ITMacroState = "none";
    ElapsedTime inTakeAndUpStateTime = new ElapsedTime();

    String TransferMacroState = "none";
    ElapsedTime TransferMacroStateTime = new ElapsedTime();

    boolean OTGrabbed = false;
    boolean ITGrabbed = false;
    ElapsedTime OT_ARM_TO_NEUTRAL_POS_TIME = new ElapsedTime();

    double prevTime = 0;

    double brakePosIT = 0;
    boolean brakingIT;

    int intakezeroPos = 0;
    int intakeMacroPos = 0;

    int rawITLPos = 0;
    int rawOTLLPos = 0;
    int rawOTLRPos = 0;

    int OTLZeroPos = 0;
    int OTRZeroPos = 0;

    double brakePosOT = 0;

    double integralIT = 0;
    double KIIT = 0.001;
    double dt = 0;

    double derivIT = 0;
    double KDIT = .01;

    double integralOT = 0;
    double KIOT = 0.0006;

    double derivOT = 0;
    double KDOT = .01;

    boolean brakingOT = true;

    DcMotor fl;
    DcMotor br;
    DcMotor fr;
    // backright drivetrain motor port 0 control
    // backleft drivetrain motor port 1 control
    // frontleft drivetrain motor port 1 expansion
    // frontright drivetrain motor port 0 expansion

    // outtake flip control servo port 0

    // to build a custom rumble effect
    Gamepad.RumbleEffect customRumbleEffect;


    public static boolean upPivotOT = true;
    double pivotTimeOT = 0;

    ControllerHandler controllerHandler;


    OpMode master;

    public enum staticVars
    {
        NONE,
        HIGH_IT_FLIP_POS_CHANGE,
        EASE_IT_FLIP_POS_CHANGE,
        LOW_IT_FLIP_POS_CHANGE,
        NEUTRAL_IT_FLIP_POS_CHANGE,
        HIGH_OT_ARM_POS_CHANGE,
        LOW_OT_ARM_POS_CHANGE,
        NEUTRAL_OT_ARM_POS_CHANGE
    }

    public void init(OpMode opMode, DcMotor frontL, DcMotor backR, DcMotor frontR)
    {
        master = opMode;
        fl = frontL;
        br = backR;
        fr = frontR;
        // outtake lifts
        outTakeLiftRight = opMode.hardwareMap.dcMotor.get("otlr");
        outTakeLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeLiftLeft = opMode.hardwareMap.dcMotor.get("otll");
        outTakeLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // outtake servos
        outTakeClaw = opMode.hardwareMap.servo.get(("otc"));
        //outTakeFlip = opMode.hardwareMap.servo.get(("otf"));
        outTakePivotRight = opMode.hardwareMap.servo.get("otpr");
        outTakePivotLeft = opMode.hardwareMap.servo.get("otpl");

        /*// color sensor
        intakeColorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");*/

        //Intake lift
        inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Intake spinners
        inTakeSpinners = opMode.hardwareMap.dcMotor.get("its");
        inTakeSpinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //inTakeFlipControl.getController().pwmEnable();

        // intake servos
        intakePivotL = opMode.hardwareMap.servo.get(("itpl"));
        intakePivotR = opMode.hardwareMap.servo.get(("itpr"));

        //outTakeFlip.setPosition(STRAIGHT_OT_FLIP_POS);
        outTakePivotRight.setPosition(LOW_OT_ARM_POSR);
        outTakePivotLeft.setPosition(LOW_OT_ARM_POSL);

        brakePosIT = inTakeLift.getCurrentPosition();
        brakePosOT = (outTakeLiftLeft.getCurrentPosition() + outTakeLiftLeft.getCurrentPosition()) / 2.0;

        //intakePivotL.setPosition(UP_IT_FLIP_POS - .15);

        //hyperServo = opMode.hardwareMap.servo.get("elbowR");


        update();
        setOutTakeZeroPos(0);
        setIntakeZeroPos(0);
        update();
        setTargetITLPos(itlPos);
        intakeMacroPos = itlPos;
        setMacroBrakeValsOT();

        opMode.telemetry.addData("itl", itlPos);

        //outTakePivotLeft.setPosition(.55);
        intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL);
        intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR);

    }

    public void initPastFirstFrame(ControllerHandler CH)
    {
        controllerHandler = CH;
    }



    public void powerOTPIDToTarget()
    {
        double errorL = targetOTLPosL - leftOTLPos;
        double errorR = targetOTLPosR - rightOTLPos;

        double proportionalL = errorL / 100.0 * kpOT;
        double proportionalR = errorR / 100.0 * kpOT;

        integralOT += (dt) * (errorR + errorL) / 2 * KIOT;

        double powerL = proportionalL + integralOT;
        double powerR = proportionalR + integralOT;

        master.telemetry.addData("pl", proportionalL);
        master.telemetry.addData("pr", proportionalR);
        master.telemetry.addData("il", integralOT);
        master.telemetry.addData("tl", targetOTLPosL);

        outTakeLiftLeft.setPower(powerL);
        outTakeLiftRight.setPower(powerR);
    }

    public void powerITPIDToTarget()
    {
        double error = targetITLiftPos - itlPos;
        double proportional = error / 100.0 * kpIT;

        inTakeLift.setPower(proportional);
    }

    public void powerITPIDToTarget(double power)
    {
        double error = targetITLiftPos - itlPos;
        double proportional = error / 100.0 * kpIT;

        inTakeLift.setPower(proportional * power);
    }

    // This method sets the "zero" position of the intake lift along with an offset, meaning it sets
    // the current position of the intake motor plus the offset to zero.
    public void setIntakeZeroPos(int offset)
    {
        intakezeroPos = rawITLPos + offset;
        intakeMacroPos = intakeMacroPos - intakezeroPos;
        itlPos = 0;
    }

    public void setIntakeMacroPos(int pos)
    {
        intakeMacroPos = pos;
    }

    public void setOutTakeZeroPos(int offset)
    {
        OTLZeroPos = rawOTLLPos + offset;
        OTRZeroPos = rawOTLRPos + offset;
        leftOTLPos = 0;
        rightOTLPos = 0;
    }

    public void setMacroBrakeValsOT()
    {
        targetOTLPosR = rightOTLPos;
        targetOTLPosL = leftOTLPos;
    }


    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseOuttakeLift()
    {
        double leftStickY = -master.gamepad2.left_stick_y;
        if (Math.abs(leftStickY) > .05) {
            outTakeLiftLeft.setPower(leftStickY);
            outTakeLiftRight.setPower(leftStickY);
            setMacroBrakeValsOT();
            approachingTarOT = false;
        }
        else if (!approachingTarOT)
        {
            resetMacroVals(false);
            approachingTarOT = true;
        }

        if (approachingTarOT)
        {
            powerOTPIDToTarget();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseIntakeLift()
    {
        double rightStickX = master.gamepad2.right_stick_x;
        master.telemetry.addData("rsx", rightStickX);


        if (Math.abs(rightStickX) > .05) {
            double trueVal = Math.signum(rightStickX) * Math.pow(rightStickX, 2) * .7;
            inTakeLift.setPower(trueVal * .7);
            setTargetITLPos(itlPos);
            approachingTarIT = false;
            kpIT = .1;
        }
        else if (!approachingTarIT)
        {
            resetMacroVals(true);
            approachingTarIT = true;
        }

        if (approachingTarIT)
        {
            powerITPIDToTarget();
        }

        master.telemetry.addData("itlpos", itlPos);
        master.telemetry.addData("tarpos", targetITLiftPos);

        if (master.gamepad2.x)
        {
            setIntakeZeroPos(0);
            intakeMacroPos = itlPos;
            setTargetITLPos(intakeMacroPos);
        }

    }

    public void resetMacroVals(boolean intake){
        if (intake)
        {
            kpIT = .3;
        }
        else
        {
            kpOT = .2;
            integralOT = 0;
        }
    }

    public void setTargetOTLPosR(int tarPos)
    {
        targetOTLPosR = tarPos;
    }
    public void setTargetOTLPosL(int tarPos)
    {
        targetOTLPosL = tarPos;
    }
    public void setTargetITLPos(int tarPos)
    {
        targetITLiftPos = tarPos;
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOuttakeArmToNeutralPos()
    {
        if (master.gamepad2.b && OT_ARM_TO_NEUTRAL_POS_TIME.milliseconds() > 400) {
            if (isStraight) {
                //outTakeFlip.setPosition(STRAIGHT_OT_FLIP_POS); // has to be tuned, the down position to flip the outtake to in order to allow it to pass through the slides
                OT_ARM_TO_NEUTRAL_POS_TIME.reset();
                isStraight = false;
            } else {
                //outTakeFlip.setPosition(BENT_OT_FLIP_POS); // has to be tuned, the down position to flip the outtake to in order to allow it to pass through the slides
                OT_ARM_TO_NEUTRAL_POS_TIME.reset();
                isStraight = true;
            }
        }
    }

    public void moveIntakeDown()
    {
        inTakeStateTime.reset();
        while (inTakeStateTime.milliseconds() < 500)
        {
            intakePivotL.setPosition(EASE_IT_FLIP_POSL);
            intakePivotR.setPosition(EASE_IT_FLIP_POSR);
        }
        intakePivotL.setPosition(LOW_IT_FLIP_POSL);
        intakePivotR.setPosition(LOW_IT_FLIP_POSR);
    }

    public void powerSpinners(double power)
    {
        inTakeSpinners.setPower(-power);
    }


    public void moveOTLiftEncoder(double power, int tarPos, double timeOut)
    {
        update();
        ElapsedTime time = new ElapsedTime();
        resetMacroVals(false);
        setTargetOTLPosR(rightOTLPos + tarPos);
        setTargetOTLPosL(leftOTLPos + tarPos);
        kpOT = .2;
        while (Math.abs(Math.abs(targetOTLPosL) - Math.abs(leftOTLPos)) > 75 && time.milliseconds() < timeOut)
        {
            update();
            powerOTPIDToTarget();
            master.telemetry.update();
        }
        //resetMacroVals(false);


    }

    public void setOTLiftPower(double power)
    {
        outTakeLiftLeft.setPower(power);
        outTakeLiftRight.setPower(power);
    }

    public void moveITLiftEncoder(double power, int tarPos, double timeOut)
    {
        update();
        ElapsedTime time = new ElapsedTime();
        setTargetITLPos(itlPos + tarPos);
        resetMacroVals(true);
        while (Math.abs(Math.abs(targetITLiftPos) - Math.abs(itlPos)) > 25 && time.seconds() < timeOut)
        {
            update();
            powerITPIDToTarget();
        }
    }

    public void powerITLift(double power)
    {
        inTakeLift.setPower(-power);
    }



    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeClawGrab(){
        if (master.gamepad2.y && totalTime.milliseconds() > lastClawTime + 500)
        {
            master.telemetry.addData("tt", totalTime.milliseconds());
            if (!OTGrabbed)
            {
                lastClawTime = totalTime.milliseconds();
                outTakeClaw.setPosition(GRAB_CLAW_POS);
                OTGrabbed = true;
            }
            else
            {
                lastClawTime = totalTime.milliseconds();
                outTakeClaw.setPosition(OPEN_CLAW_POS);
                OTGrabbed = false;
            }
        }
    }


    public void switchITMacroState(String newstate)
    {
        TransferMacroStateTime.reset(); //e
        ITMacroState = newstate;
    }

    public void transferMacro() {
        //outTakeClaw.setPosition(GRAB_CLAW_POS); // set the outtake claw to its most closed position

        if (master.gamepad2.a && ITMacroState.equals("none")) {
            switchITMacroState("servoReady");
        }
        if (ITMacroState.equals("servoReady"))
        {
            if (TransferMacroStateTime.milliseconds() > 700) {
                intakePivotL.setPosition(HIGH_IT_FLIP_POSL);
                intakePivotR.setPosition(HIGH_IT_FLIP_POSR);
                outTakeClaw.setPosition(OPEN_CLAW_POS);
                if (TransferMacroStateTime.milliseconds() > 1300) {
                    setTargetITLPos(intakeMacroPos);
                    resetMacroVals(true);
                    switchITMacroState("closeclaw");
                }
            }
            //outTakeFlip.setPosition(NEUTRAL_OT_FLIP_POS);
            inTakeSpinners.setPower(-.4);
            outTakePivotLeft.setPosition(LOW_OT_ARM_POSL);
            outTakePivotRight.setPosition(LOW_OT_ARM_POSR);
            //outTakeFlip.setPosition(STRAIGHT_OT_FLIP_POS);
        }
        if (ITMacroState.equals("closeclaw"))
        {
            if (TransferMacroStateTime.milliseconds() > 1500)
            {
                outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL);
                outTakePivotRight.setPosition(HIGH_OT_ARM_POSR);
                switchITMacroState("none");
            }
            else if (TransferMacroStateTime.milliseconds() > 1000) {
                resetMacroVals(true);// will have to tune, meant to be a position to get the outtake flipped to the lowest possible position (arm ready to be moved down)
                setTargetITLPos(intakeMacroPos + 400);
            }
            else if (TransferMacroStateTime.milliseconds() > 700) {
                OTGrabbed = true;
                outTakeClaw.setPosition(GRAB_CLAW_POS);
            }
            else {
                inTakeSpinners.setPower(0);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    public void setIntakePivot()
    {
        if (master.gamepad2.dpad_right)
        {
            inTakeState = "low";
            inTakeStateTime.reset();
        }
        else if (master.gamepad2.dpad_left)
        {
            inTakeState = "high";
            intakePivotL.setPosition(HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
            intakePivotR.setPosition(HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
            prevValChanged = staticVars.HIGH_IT_FLIP_POS_CHANGE;
            inTakeStateTime.reset();
            inTakeState = "none";
        }
        else if (master.gamepad2.dpad_up)
        {
            inTakeState = "up";
            intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
            intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
            prevValChanged = staticVars.NEUTRAL_IT_FLIP_POS_CHANGE;
            inTakeStateTime.reset();
            inTakeState = "none";
        }

        if (inTakeState.equals("low"))
        {
            if (inTakeStateTime.milliseconds() > 500) {
                intakePivotL.setPosition(LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
                intakePivotR.setPosition(LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
                prevValChanged = staticVars.LOW_IT_FLIP_POS_CHANGE;
                inTakeState = "none";
            }
            else
            {
                intakePivotL.setPosition(LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE - .04);
                intakePivotR.setPosition(LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE + .04);
            }
        }

        if (master.gamepad2.right_bumper)
        {
            intakePivotL.setPosition(HIGH_IT_FLIP_POSL + .02);
            intakePivotR.setPosition(HIGH_IT_FLIP_POSR - .02);
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    public void setOuttakePivot()
    {
        if (master.gamepad2.dpad_down && transferringOTPivot && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
            outTakePivotRight.setPosition(HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
            prevValChanged = staticVars.HIGH_OT_ARM_POS_CHANGE;
            transferringOTPivot = false;
            otPivotTime.reset();
        }
        else if (master.gamepad2.dpad_down && !transferringOTPivot && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
            outTakePivotRight.setPosition(LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
            prevValChanged = staticVars.LOW_OT_ARM_POS_CHANGE;
            transferringOTPivot = true;
            otPivotTime.reset();
        }
        else if (master.gamepad2.left_bumper && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(NEUTRAL_OT_ARM_POSL + NEUTRAL_OT_ARM_POSL_CHANGE);
            outTakePivotRight.setPosition(NEUTRAL_OT_ARM_POSR + NEUTRAL_OT_ARM_POSR_CHANGE);
            prevValChanged = staticVars.NEUTRAL_OT_ARM_POS_CHANGE;
            transferringOTPivot = true;
            otPivotTime.reset();
        }
        else if (master.gamepad2.x && otPivotTime.milliseconds() > 300)
        {
           // outTakePivotLeft.setPosition(.55);
            //transferringOTPivot = true;
            //otPivotTime.reset();
        }
    }
    //////////////////////////////////////////////////////////////////////////////
    public void setIntakeSpinners() {
        if (master.gamepad2.right_trigger > 0.1) {
            inTakeSpinners.setPower(-master.gamepad2.right_trigger);
        } else if (master.gamepad2.left_trigger > 0.1) {
            inTakeSpinners.setPower(master.gamepad2.left_trigger);
        } else
        {
            inTakeSpinners.setPower(0);
        }
    }

    public void changeStaticVals()
    {
        if (controllerHandler.isGP1RightBumperPressed1Frame()) {
            if (prevValChanged == staticVars.HIGH_IT_FLIP_POS_CHANGE) {
                HIGH_IT_FLIP_POSL_CHANGE -= .01;
                intakePivotL.setPosition(HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
                HIGH_IT_FLIP_POSR_CHANGE += .01;
                intakePivotR.setPosition(HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("HIGH IT FLIP POS CHANGE", Math.abs(HIGH_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("HIGH IT FLIP POSL", HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("HIGH IT FLIP POSR", HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.EASE_IT_FLIP_POS_CHANGE) {
                EASE_IT_FLIP_POSL_CHANGE -= .01;
                intakePivotL.setPosition(EASE_IT_FLIP_POSL + EASE_IT_FLIP_POSL_CHANGE);
                EASE_IT_FLIP_POSR_CHANGE += .01;
                intakePivotR.setPosition(EASE_IT_FLIP_POSR + EASE_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("EASE IT FLIP POS CHANGE", Math.abs(EASE_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("EASE IT FLIP POSL", EASE_IT_FLIP_POSL + EASE_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("EASE IT FLIP POSR", EASE_IT_FLIP_POSR + EASE_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.LOW_IT_FLIP_POS_CHANGE) {
                LOW_IT_FLIP_POSL_CHANGE -= .01;
                intakePivotL.setPosition(LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
                LOW_IT_FLIP_POSR_CHANGE += .01;
                intakePivotR.setPosition(LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("LOW IT FLIP POS CHANGE", Math.abs(LOW_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("LOW IT FLIP POSL", LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("LOW IT FLIP POSR", LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.NEUTRAL_IT_FLIP_POS_CHANGE) {
                NEUTRAL_IT_FLIP_POSL_CHANGE -= .01;
                intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
                NEUTRAL_IT_FLIP_POSR_CHANGE += .01;
                intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("NEUTRAL IT FLIP POS CHANGE", Math.abs(NEUTRAL_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("NEUTRAL IT FLIP POSL", NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("NEUTRAL IT FLIP POSR", NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.HIGH_OT_ARM_POS_CHANGE) {
                HIGH_OT_ARM_POSL_CHANGE -= .01;
                outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
                HIGH_OT_ARM_POSR_CHANGE += .01;
                outTakePivotRight.setPosition(HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
                master.telemetry.addData("HIGH OT ARM POS CHANGE", Math.abs(HIGH_OT_ARM_POSL_CHANGE));
                master.telemetry.addData("HIGH OT ARM POSL", HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
                master.telemetry.addData("HIGH OT ARM POSR", HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.LOW_OT_ARM_POS_CHANGE) {
                LOW_OT_ARM_POSL_CHANGE -= .01;
                outTakePivotLeft.setPosition(LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
                LOW_OT_ARM_POSR_CHANGE += .01;
                outTakePivotRight.setPosition(LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
                master.telemetry.addData("LOW OT ARM POS CHANGE", Math.abs(LOW_OT_ARM_POSL_CHANGE));
                master.telemetry.addData("LOW OT ARM POSL", LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
                master.telemetry.addData("LOW OT ARM POSR", LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.NEUTRAL_OT_ARM_POS_CHANGE) {
                NEUTRAL_OT_ARM_POSL_CHANGE -= .01;
                outTakePivotLeft.setPosition(NEUTRAL_OT_ARM_POSL + NEUTRAL_OT_ARM_POSL_CHANGE);
                NEUTRAL_OT_ARM_POSR_CHANGE += .01;
                outTakePivotRight.setPosition(NEUTRAL_OT_ARM_POSR + NEUTRAL_OT_ARM_POSR_CHANGE);
                master.telemetry.addData("NEUTRAL OT ARM POS CHANGE", Math.abs(NEUTRAL_OT_ARM_POSL_CHANGE));
                master.telemetry.addData("NEUTRAL OT ARM POSL", NEUTRAL_OT_ARM_POSL + NEUTRAL_OT_ARM_POSL_CHANGE);
                master.telemetry.addData("NEUTRAL OT ARM POSR", NEUTRAL_OT_ARM_POSR + NEUTRAL_OT_ARM_POSR_CHANGE);
            }
            master.telemetry.addData("prev", prevValChanged);
        }
        else if (controllerHandler.isGP1LeftBumperPressed1Frame()) {
            if (prevValChanged == staticVars.HIGH_IT_FLIP_POS_CHANGE) {
                HIGH_IT_FLIP_POSL_CHANGE += .01;
                intakePivotL.setPosition(HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
                HIGH_IT_FLIP_POSR_CHANGE -= .01;
                intakePivotR.setPosition(HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("HIGH IT FLIP POS CHANGE", Math.abs(HIGH_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("HIGH IT FLIP POSL", HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("HIGH IT FLIP POSR", HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.EASE_IT_FLIP_POS_CHANGE) {
                EASE_IT_FLIP_POSL_CHANGE += .01;
                intakePivotL.setPosition(EASE_IT_FLIP_POSL + EASE_IT_FLIP_POSL_CHANGE);
                EASE_IT_FLIP_POSR_CHANGE -= .01;
                intakePivotR.setPosition(EASE_IT_FLIP_POSR + EASE_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("EASE IT FLIP POS CHANGE", Math.abs(EASE_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("EASE IT FLIP POSL", EASE_IT_FLIP_POSL + EASE_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("EASE IT FLIP POSR", EASE_IT_FLIP_POSR + EASE_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.LOW_IT_FLIP_POS_CHANGE) {
                LOW_IT_FLIP_POSL_CHANGE += .01;
                intakePivotL.setPosition(LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
                LOW_IT_FLIP_POSR_CHANGE -= .01;
                intakePivotR.setPosition(LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("LOW IT FLIP POS CHANGE", Math.abs(LOW_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("LOW IT FLIP POSL", LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("LOW IT FLIP POSR", LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.NEUTRAL_IT_FLIP_POS_CHANGE) {
                NEUTRAL_IT_FLIP_POSL_CHANGE += .01;
                intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
                NEUTRAL_IT_FLIP_POSR_CHANGE -= .01;
                intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
                master.telemetry.addData("NEUTRAL IT FLIP POS CHANGE", Math.abs(NEUTRAL_IT_FLIP_POSR_CHANGE));
                master.telemetry.addData("NEUTRAL IT FLIP POSL", NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
                master.telemetry.addData("NEUTRAL IT FLIP POSR", NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.HIGH_OT_ARM_POS_CHANGE) {
                HIGH_OT_ARM_POSL_CHANGE += .01;
                outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
                HIGH_OT_ARM_POSR_CHANGE -= .01;
                outTakePivotRight.setPosition(HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
                master.telemetry.addData("HIGH OT ARM POS CHANGE", Math.abs(HIGH_OT_ARM_POSL_CHANGE));
                master.telemetry.addData("HIGH OT ARM POSL", HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
                master.telemetry.addData("HIGH OT ARM POSR", HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
            }
            if (prevValChanged == staticVars.LOW_OT_ARM_POS_CHANGE) {
                LOW_OT_ARM_POSL_CHANGE += .01;
                outTakePivotLeft.setPosition(LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
                LOW_OT_ARM_POSR_CHANGE -= .01;
                outTakePivotRight.setPosition(LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
                master.telemetry.addData("LOW OT ARM POS CHANGE", Math.abs(LOW_OT_ARM_POSL_CHANGE));
                master.telemetry.addData("LOW OT ARM POSL", LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
                master.telemetry.addData("LOW OT ARM POSR", LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
            }
            master.telemetry.addData("prev", prevValChanged);
            master.telemetry.addLine("in the left bumper");
        }

        if (LOW_OT_ARM_POSL_CHANGE != 0)
        {
            master.telemetry.addData("LOW OT ARM POS CHANGE", Math.abs(LOW_OT_ARM_POSL_CHANGE));
            master.telemetry.addData("LOW OT ARM POSL", LOW_OT_ARM_POSL + LOW_OT_ARM_POSL_CHANGE);
            master.telemetry.addData("LOW OT ARM POSR", LOW_OT_ARM_POSR + LOW_OT_ARM_POSR_CHANGE);
        }
        if (HIGH_OT_ARM_POSL_CHANGE != 0)
        {
            master.telemetry.addData("HIGH OT ARM POS CHANGE", Math.abs(HIGH_OT_ARM_POSL_CHANGE));
            master.telemetry.addData("HIGH OT ARM POSL", HIGH_OT_ARM_POSL + HIGH_OT_ARM_POSL_CHANGE);
            master.telemetry.addData("HIGH OT ARM POSR", HIGH_OT_ARM_POSR + HIGH_OT_ARM_POSR_CHANGE);
        }
        if (NEUTRAL_OT_ARM_POSL_CHANGE != 0)
        {
            master.telemetry.addData("NEUTRAL OT ARM POS CHANGE", Math.abs(NEUTRAL_OT_ARM_POSL_CHANGE));
            master.telemetry.addData("NEUTRAL OT ARM POSL", NEUTRAL_OT_ARM_POSL + NEUTRAL_OT_ARM_POSL_CHANGE);
            master.telemetry.addData("NEUTRAL OT ARM POSR", NEUTRAL_OT_ARM_POSR + NEUTRAL_OT_ARM_POSR_CHANGE);
        }
        if (HIGH_IT_FLIP_POSL_CHANGE != 0)
        {
            master.telemetry.addData("HIGH IT FLIP POS CHANGE", Math.abs(HIGH_IT_FLIP_POSR_CHANGE));
            master.telemetry.addData("HIGH IT FLIP POSL", HIGH_IT_FLIP_POSL + HIGH_IT_FLIP_POSL_CHANGE);
            master.telemetry.addData("HIGH IT FLIP POSR", HIGH_IT_FLIP_POSR + HIGH_IT_FLIP_POSR_CHANGE);
        }
        if (EASE_IT_FLIP_POSL_CHANGE != 0)
        {
            master.telemetry.addData("EASE IT FLIP POS CHANGE", Math.abs(EASE_IT_FLIP_POSR_CHANGE));
            master.telemetry.addData("EASE IT FLIP POSL", EASE_IT_FLIP_POSL + EASE_IT_FLIP_POSL_CHANGE);
            master.telemetry.addData("EASE IT FLIP POSR", EASE_IT_FLIP_POSR + EASE_IT_FLIP_POSR_CHANGE);
        }
        if (LOW_IT_FLIP_POSL_CHANGE != 0)
        {
            master.telemetry.addData("LOW IT FLIP POS CHANGE", Math.abs(LOW_IT_FLIP_POSR_CHANGE));
            master.telemetry.addData("LOW IT FLIP POSL", LOW_IT_FLIP_POSL + LOW_IT_FLIP_POSL_CHANGE);
            master.telemetry.addData("LOW IT FLIP POSR", LOW_IT_FLIP_POSR + LOW_IT_FLIP_POSR_CHANGE);
        }
        if (NEUTRAL_IT_FLIP_POSL_CHANGE != 0)
        {
            master.telemetry.addData("NEUTRAL IT FLIP POS CHANGE", Math.abs(NEUTRAL_IT_FLIP_POSR_CHANGE));
            master.telemetry.addData("NEUTRAL IT FLIP POSL", NEUTRAL_IT_FLIP_POSL + NEUTRAL_IT_FLIP_POSL_CHANGE);
            master.telemetry.addData("NEUTRAL IT FLIP POSR", NEUTRAL_IT_FLIP_POSR + NEUTRAL_IT_FLIP_POSR_CHANGE);
        }
        master.telemetry.update();
    }

    //////////////////////////////////////////////////////////////////////////////
    public void runTesting()
    {
        /*if (master.gamepad2.a)
        {
            // up
            inTakeClaw.setPosition(.76);
        }*/
        /*if (master.gamepad2.b)
        {
            //grab
            intakePivotL.setPosition(.3);
            intakePivotR.setPosition(.3);
            //inTakeFlipControl.getController().setServoPosition();
        }
        if (master.gamepad2.dpad_up)
        {
            //put in transfer
            intakePivotL.setPosition(.5);
            intakePivotR.setPosition(.5);
        }*/
        /*if (master.gamepad2.dpad_down)
        {
            //put in transfer
            inTakeRotator.setPosition(.32);
        }*/
        /*if (master.gamepad2.dpad_left)
        {
            inTakeRotator.setPosition(0);
        }
        if (master.gamepad2.x)
        {
            intakePivotL.setPosition(.7);
            intakePivotR.setPosition(.7);
        }
        if (master.gamepad2.y)
        {
            inTakeClaw.setPosition(1);
        }*/
        if (master.gamepad2.left_bumper)
        {
            outTakeLiftLeft.setPower(.8);
            outTakeLiftRight.setPower(.8);
        }
        else if (master.gamepad2.right_bumper)
        {
            outTakeLiftLeft.setPower(-.8);
            outTakeLiftRight.setPower(-.8);
        }
        else
        {
            outTakeLiftLeft.setPower(0);
            outTakeLiftRight.setPower(0);
        }
        if (master.gamepad2.left_trigger > .1)
        {
            inTakeLift.setPower(.8);
        }
        else if (master.gamepad2.right_trigger > .1)
        {
            inTakeLift.setPower(-.8);
        }
        else {
            inTakeLift.setPower(0);
        }
    }

    public void update()
    {
        dt = totalTime.seconds() - prevTime;
        rawITLPos = -fl.getCurrentPosition();
        rawOTLLPos = -br.getCurrentPosition();
        rawOTLRPos = -fr.getCurrentPosition();

        leftOTLPos = rawOTLLPos - OTLZeroPos;
        rightOTLPos = rawOTLRPos - OTRZeroPos;
        itlPos = rawITLPos - intakezeroPos;
        master.telemetry.addData("itlpos", itlPos);
        master.telemetry.addData("otlpos", leftOTLPos);
        master.telemetry.addData("otlposr", rightOTLPos);
        prevTime = totalTime.seconds();
    }


    public void transferMacroAuto() {
        //outTakeClaw.setPosition(GRAB_CLAW_POS); // set the outtake claw to its most closed position

        switchITMacroState("servoReady");
        while (ITMacroState.equals("servoReady"))
        {
            if (TransferMacroStateTime.milliseconds() > 700) {
                intakePivotL.setPosition(HIGH_IT_FLIP_POSL);
                intakePivotR.setPosition(HIGH_IT_FLIP_POSR);
                //outTakeClaw.setPosition(OPEN_CLAW_POS);
                if (TransferMacroStateTime.milliseconds() > 1300) {
                    setTargetITLPos(intakeMacroPos);
                    resetMacroVals(true);
                    switchITMacroState("closeclaw");
                }
            }
            inTakeSpinners.setPower(-.7);
            outTakePivotLeft.setPosition(LOW_OT_ARM_POSL);
            outTakePivotRight.setPosition(LOW_OT_ARM_POSR);
        }
        while (ITMacroState.equals("closeclaw"))
        {
            update();
            if (TransferMacroStateTime.milliseconds() > 2300)
            {
                outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL);
                outTakePivotRight.setPosition(HIGH_OT_ARM_POSR);
                switchITMacroState("none");
            }
            if (TransferMacroStateTime.milliseconds() > 1600)
            {
                setTargetITLPos(intakeMacroPos + 200);
            }
            else if (TransferMacroStateTime.milliseconds() > 1000) {
                // will have to tune, meant to be a position to get the outtake flipped to the lowest possible position (arm ready to be moved down)
                outTakeClaw.setPosition(GRAB_CLAW_POS);
            }
            else if (TransferMacroStateTime.milliseconds() > 700) {
                OTGrabbed = true;
                powerSpinners(0);

            }
            powerITPIDToTarget();
        }
    }

/*

////////////////////////////////////////////////////////////////////////////////

    public void setOutTakeFlip(){
        if (master.gamepad2.right_trigger > .1)
            outTakeFlip.setPosition(UP_OT_FLIP_POS);
        if (master.gamepad2.b)
            outTakeFlip.setPosition(MID_OT_FLIP_POS);
        if (master.gamepad2.back)
            outTakeFlip.setPosition(UP_OT_FLIP_POS + .15);
    }
    {

            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(UP_OT_FLIP_POS);
                TransferMacroStateTime.reset();
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to be a position to get the outtake flipped to the lowest possible position (arm ready to be moved down)
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(DOWN_OT_FLIP_POS); // moves whole outtake arm downwards to the position to be transferred
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to be a position to get the outtake flipped to the position so that it is ready to grab the sample
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeClaw.setPosition(OT_CLAW_GRAB); // will have to tune, meant to be a closed position to fit inside the intake but still grab the sample inside the intake
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to move the outtake arm down to get the claw in position to clamp and grab the sample
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeClaw.setPosition(OT_CLAW_GRAB); // will have to tune, clamps the sample in the intake
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(UP_OT_FLIP_POS); // will have to tune, meant to move the outtake arm up to move the claw to the position to outtake, still needs to be flipped
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(UP_OT_FLIP_POS); // will have to tune, meant to flip the outtake to the highest position to be able to drop the sample
            }
            TransferMacroStateTime.reset();
        }*/

    /*public void outTakeMacroAndTransfer()
    {
        if (master.gamepad2.a && outTakeAndUpStateTime.milliseconds() > 500 && OTMacroState.equals("none"))
        {
            outTakeAndUpStateTime.reset();
            OTMacroState = "clamp";
        }
        if (OTMacroState.equals("clamp"))
        {
            outTakeClaw.setPosition(OT_CLAW_GRAB);
            if (outTakeAndUpStateTime.milliseconds() > 200) {
                inTakeClaw.setPosition(OPEN_IT_POS);
                if (outTakeAndUpStateTime.milliseconds() > 300) {
                    outTakeAndUpStateTime.reset();
                    OTMacroState = "raise";
                }
            }
        }
        if (OTMacroState.equals("raise")) {
            outTakePivotRight.setPosition(BUCKET_OT_PIVOT_POS);
            outTakeFlip.setPosition(PLACE_OT_FLIP_POS);
            if (outTakeAndUpStateTime.milliseconds() > 750) {
                outTakeAndUpStateTime.reset();
                OTMacroState = "none";
            }
        }
    }*/


////////////////////////////////////////////////////////////////////////////////
    /*public void setOutTakePivot(){
        if (master.gamepad2.dpad_right && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            if (upPivotOT)
            {
                outTakePivotRight.setPosition(TRANSFER_OT_PIVOT_POS);
                outTakeClaw.setPosition(OT_CLAW_GRAB);
                upPivotOT = false;
            }
            else
            {
                outTakePivotRight.setPosition(BUCKET_OT_PIVOT_POS);
                outTakeClaw.setPosition(OT_CLAW_GRAB);
                outTakeFlip.setPosition(PLACE_OT_FLIP_POS);
                upPivotOT = true;
            }
            pivotTimeOT = totalTime.milliseconds();
        }
        if (master.gamepad2.x && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            //outTakeClaw.setPosition(OT_CLAW_RELEASE);
            outTakeFlip.setPosition(MID_OT_FLIP_POS);
            outTakePivotRight.setPosition(.2);
            pivotTimeOT = totalTime.milliseconds();
        }
    }*/

////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeClawGrab(){
        if (master.gamepad2.b && ITGrabTime.milliseconds() > 500) {
            if (ITGrabbed) {
                inTakeClaw.setPosition(OPEN_IT_POS);
                ITGrabbed = false;
            }
            else {
                inTakeClaw.setPosition(CLOSED_IT_POS);
                ITGrabbed = true;
            }
            ITGrabTime.reset();
        }
        if (master.gamepad2.dpad_right)
            inTakeClaw.setPosition(MIDDLE_IT_POS);
    }*/

////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeFlip(){
        if (master.gamepad2.dpad_up) {
        intakePivotL.setPosition(UP_IT_FLIP_POS);
        // intakePivotR.setPosition(UP_IT_FLIP_POS);
    }

        if (master.gamepad2.dpad_down) {
        //  intakePivotR.setPosition(DOWN_IT_FLIP_POS);
        intakePivotL.setPosition(DOWN_IT_FLIP_POS);
    }
        if (master.gamepad2.dpad_left)
    {
        //  intakePivotR.setPosition(MID_IT_FLIP_POS);
        intakePivotL.setPosition(MID_IT_FLIP_POS);
    }
}*/

////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeRotator(){
        if (master.gamepad2.left_bumper)
            inTakeRotator.setPosition(PAR_IT_POS);
        if (master.gamepad2.right_bumper)
            inTakeRotator.setPosition(PERP_IT_POS);
    }*/

    ////////////////////////////////////////////////////////////////////////////////
    /*public void rumbleControllerWhenCorrectPieceIntook()
    {
        while(true)
        {
            intakeColorSensor.
            master.telemetry.addData()
        }
    }*/



}
