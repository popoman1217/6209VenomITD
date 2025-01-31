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
    public static double LOW_OT_ARM_POSL = 0.23;
    public static double NEUTRAL_OT_ARM_POSL = 0.55;

    public static double HIGH_OT_ARM_POSR = 0.03;
    public static double LOW_OT_ARM_POSR = .4;
    public static double NEUTRAL_OT_ARM_POSR = .08;

    public static double STRAIGHT_OT_FLIP_POS = 0.6;
    public static double BENT_OT_FLIP_POS = .3;
    //public static double NEUTRAL_OT_FLIP_POS = .8;


    // done
    public static double GRAB_CLAW_POS = 0.06;
    public static double OPEN_CLAW_POS = 0.3;
    public static double NEUTRAL_CLAW_POS = 0.2;

    public static double HIGH_IT_FLIP_POSR = .65;
    public static double LOW_IT_FLIP_POSR = 0.05;
    public static double HIGH_IT_FLIP_POSL = 0.4;
    public static double LOW_IT_FLIP_POSL = 0.96;
    public static double NEUTRAL_IT_FLIP_POSR = 0.4;
    public static double NEUTRAL_IT_FLIP_POSL = 0.58;

    ElapsedTime intakeToTransfer = new ElapsedTime();

    String ITMacroState = "none";
    ElapsedTime inTakeAndUpStateTime = new ElapsedTime();

    String TransferMacroState = "none";
    ElapsedTime TransferMacroStateTime = new ElapsedTime();

    boolean OTGrabbed = false;
    boolean ITGrabbed = false;
    ElapsedTime OT_ARM_TO_NEUTRAL_POS_TIME = new ElapsedTime();

    double brakePosIT = 0;
    boolean brakingIT;

    int intakezeroPos = 0;
    int OTLZeroPos = 0;
    int OTRZeroPos = 0;

    double brakePosOT = 0;
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


    OpMode master;

    public void init(OpMode opMode, DcMotor frontL, DcMotor backR, DcMotor frontR)
    {
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
        outTakePivotRight.setPosition(NEUTRAL_OT_ARM_POSR);
        outTakePivotLeft.setPosition(NEUTRAL_OT_ARM_POSL);

        brakePosIT = inTakeLift.getCurrentPosition();
        brakePosOT = (outTakeLiftLeft.getCurrentPosition() + outTakeLiftLeft.getCurrentPosition()) / 2.0;

        //intakePivotL.setPosition(UP_IT_FLIP_POS - .15);

        //hyperServo = opMode.hardwareMap.servo.get("elbowR");

        leftOTLPos = br.getCurrentPosition();
        rightOTLPos = fr.getCurrentPosition();
        itlPos = fl.getCurrentPosition();

        targetITLiftPos = itlPos;

        intakezeroPos = itlPos;
        OTLZeroPos = leftOTLPos;
        OTRZeroPos = rightOTLPos;
        opMode.telemetry.addData("itl", itlPos);
        targetOTLPosL = leftOTLPos;
        targetOTLPosR = rightOTLPos;

        //outTakePivotLeft.setPosition(.55);
        intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL);
        intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR);
        master = opMode;
    }


    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseOuttakeLift()
    {
        double leftStickY = -master.gamepad2.left_stick_y;
        if (Math.abs(leftStickY) > .05) {
            kpOT = .08;
            outTakeLiftLeft.setPower(leftStickY);
            outTakeLiftRight.setPower(leftStickY);
            targetOTLPosL = leftOTLPos;
            targetOTLPosR = rightOTLPos;
            approachingTarOT = false;
        }
        else
        {
            approachingTarOT = true;
        }

        if (approachingTarOT)
        {
            outTakeLiftRight.setPower((targetOTLPosR - rightOTLPos) / 100.0 * kpOT);
            outTakeLiftLeft.setPower((targetOTLPosL - leftOTLPos) / 100.0 * kpOT);
        }



    }

    public void setOTBrake()
    {
        kpOT = .08;
        outTakeLiftRight.setPower((targetOTLPosR - rightOTLPos) / 100.0 * kpOT);
        outTakeLiftLeft.setPower((targetOTLPosL - leftOTLPos) / 100.0 * kpOT);
    }

    public void setMacroBrakeValsOT()
    {
        update();
        targetOTLPosR = rightOTLPos;
        targetOTLPosL = leftOTLPos;
    }
    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseIntakeLift()
    {
        double rightStickX = -master.gamepad2.right_stick_x;
        master.telemetry.addData("rsx", rightStickX);

        if (master.gamepad2.x)
        {
            intakezeroPos = itlPos;
        }

        if (Math.abs(rightStickX) > .05) {
            inTakeLift.setPower(rightStickX * .7);
            targetITLiftPos = itlPos;
            approachingTarIT = false;
            kpIT = .1;
        }
        else
        {
            approachingTarIT = true;
        }

        if (approachingTarIT)
        {
            inTakeLift.setPower(-(targetITLiftPos - itlPos) / 100.0 * kpIT);
        }

        master.telemetry.addData("itlpos", itlPos);
        master.telemetry.addData("tarpos", targetITLiftPos);

    }

    public void setMacroVals(int tarPos, boolean intake){
        if (intake)
        {
            targetITLiftPos = tarPos;
            kpIT = .17;
        }
        else
        {
            targetOTLPosR = tarPos;
            targetOTLPosL = tarPos;
            kpOT = .2;
        }
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


    public void moveOTLiftEncoder(double power, int tarPos, double timeOut)
    {
        update();
        ElapsedTime time = new ElapsedTime();
        targetOTLPosL = leftOTLPos + tarPos;
        targetOTLPosR = rightOTLPos + tarPos;
        while (Math.abs(Math.abs(targetOTLPosL) - Math.abs(leftOTLPos)) > 100 && time.milliseconds() < timeOut)
        {
            double sign = Math.signum(targetOTLPosL - leftOTLPos);
            outTakeLiftLeft.setPower(Math.signum(tarPos) * Math.min(power, Math.pow(targetOTLPosL - leftOTLPos, 1) / 100 * power * sign * .3));
            outTakeLiftRight.setPower(Math.signum(tarPos) * Math.min(power, Math.pow(targetOTLPosL - leftOTLPos, 1) / 100 * power * sign * .3));
            update();
        }


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
                if (TransferMacroStateTime.milliseconds() > 1300)
                    switchITMacroState("closeclaw");
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
                    setMacroVals(intakezeroPos - 400, true);// will have to tune, meant to be a position to get the outtake flipped to the lowest possible position (arm ready to be moved down)
            }
            else if (TransferMacroStateTime.milliseconds() > 700)
                outTakeClaw.setPosition(GRAB_CLAW_POS);
            else {
                setMacroVals(intakezeroPos, true);
                inTakeSpinners.setPower(0);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    public void setIntakePivot()
    {
        if (master.gamepad2.dpad_right)
        {
            intakePivotL.setPosition(LOW_IT_FLIP_POSL);
            intakePivotR.setPosition(LOW_IT_FLIP_POSR);
        }
        else if (master.gamepad2.dpad_left)
        {
            intakePivotL.setPosition(HIGH_IT_FLIP_POSL);
            intakePivotR.setPosition(HIGH_IT_FLIP_POSR);
        }
        else if (master.gamepad2.dpad_up)
        {
            intakePivotL.setPosition(NEUTRAL_IT_FLIP_POSL);
            intakePivotR.setPosition(NEUTRAL_IT_FLIP_POSR);
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    public void setOuttakePivot()
    {
        if (master.gamepad2.dpad_down && transferringOTPivot && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(HIGH_OT_ARM_POSL);
            outTakePivotRight.setPosition(HIGH_OT_ARM_POSR);
            transferringOTPivot = false;
            otPivotTime.reset();
        }
        else if (master.gamepad2.dpad_down && !transferringOTPivot && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(LOW_OT_ARM_POSL);
            outTakePivotRight.setPosition(LOW_OT_ARM_POSR);
            transferringOTPivot = true;
            otPivotTime.reset();
        }
        else if (master.gamepad2.left_bumper && otPivotTime.milliseconds() > 300)
        {
            outTakePivotLeft.setPosition(NEUTRAL_OT_ARM_POSL);
            outTakePivotRight.setPosition(NEUTRAL_OT_ARM_POSR);
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
    //////////////////////////////////////////////////////////////////////////////
    /*public void transfer() throws InterruptedException{
        if (intakeToTransfer.milliseconds() > 0 && intakeToTransfer.milliseconds() < 200)

            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 200 && intakeToTransfer.milliseconds() < 400)
        {
            intakePivotL.setPosition(1); // pos of intake so that it is lifted
            intakePivotR.setPosition(1);
        }
        if (intakeToTransfer.milliseconds() > 400 && intakeToTransfer.milliseconds() < 600)
        {
            intakePivotL.setPosition(0); // pos of intake when it is in the transfer position
            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 600 && intakeToTransfer.milliseconds() < 800)
        {
            outTakeFlip.setPosition(0);
            intakePivotL.setPosition(0); // pos of intake so that it is lifted
            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 800 && intakeToTransfer.milliseconds() < 1000)
        {
            outTakePivotRight.setPosition(0); // pivot outtake so that it is in the pos where it drops the pixel
            outTakePivotLeft.setPosition(0);
        }
    }*/
    //////////////////////////////////////////////////////////////////////////////
    /*public void slidePosHigh() {
        outTakeLiftLeft.setTargetPosition(1); // encoder position of highest position slides need to be
        outTakeLiftRight.setTargetPosition(1);

    }*/
    /*
    //////////////////////////////////////////////////////////////////////////////
    public void servotesting() {
        double rotatorConstant = 0.0;
        if (master.gamepad1.x) {
            rotatorConstant += 0.1;
            intakePivotL.setPosition(rotatorConstant);
            intakePivotR.setPosition(rotatorConstant);
            master.telemetry.update();
            master.telemetry.addData("Servo position", rotatorConstant);
        }

    }

     */
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
        leftOTLPos = br.getCurrentPosition();
        rightOTLPos = fr.getCurrentPosition();
        itlPos = -fl.getCurrentPosition();
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
