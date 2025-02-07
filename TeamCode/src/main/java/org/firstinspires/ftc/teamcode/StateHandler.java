package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StateHandler {
    public OpMode master;
    String state = "beginning";
    String prevState = "none";
    ElapsedTime stateTime = new ElapsedTime();
    boolean firstStateFrame = true;

    public void init(OpMode opMode){
        master = opMode;
    }

    public enum CoRoutines{
        OTPID,
        ITPID,
        None,
    }

    public void switchState(String newState)
    {
        master.telemetry.addLine("switching state");
        master.telemetry.addData("old state", state);
        prevState = state;
        state = newState;
        stateTime.reset();
        firstStateFrame = true;
        master.telemetry.addData("new state", state);
        master.telemetry.update();
    }

    public void switchState(String newState, long delay, Mechanisms mechanisms, CoRoutines[] coRoutines)
    {
        ElapsedTime delayTime = new ElapsedTime();
        master.telemetry.addLine("switching state");
        master.telemetry.addData("old state", state);
        prevState = state;
        state = newState;
        stateTime.reset();
        firstStateFrame = true;
        master.telemetry.addData("new state", state);
        master.telemetry.addData("sleeping X seconds", delay);
        boolean isOTPID = false;
        boolean isITPID = false;
        for (CoRoutines curCR : coRoutines)
        {
            if (curCR == CoRoutines.None) {
                isOTPID = false;
                isITPID = false;
            }
            if (curCR == CoRoutines.ITPID)
                isITPID = true;
            if (curCR == CoRoutines.OTPID)
                isOTPID = true;
        }
        while (delayTime.milliseconds() < delay)
        {
            mechanisms.update();
            if (isITPID)
                mechanisms.powerITPIDToTarget();
            if (isOTPID)
                mechanisms.powerOTPIDToTarget();

        }
        master.telemetry.update();
    }

    public void switchState(String newState, long delay, Mechanisms mechanisms, CoRoutines coRoutine)
    {
        ElapsedTime delayTime = new ElapsedTime();
        master.telemetry.addLine("switching state");
        master.telemetry.addData("old state", state);
        prevState = state;
        state = newState;
        stateTime.reset();
        firstStateFrame = true;
        master.telemetry.addData("new state", state);
        master.telemetry.addData("sleeping X seconds", delay);
        while (delayTime.milliseconds() < delay)
        {
            mechanisms.update();
            if (coRoutine == CoRoutines.ITPID)
                mechanisms.powerITPIDToTarget();
            if (coRoutine == CoRoutines.OTPID)
                mechanisms.powerOTPIDToTarget();

        }
        master.telemetry.update();
    }

    public void stateUpdate()
    {
        master.telemetry.addData("old state", prevState);
        master.telemetry.addData("new state", state);
        master.telemetry.addData("state time", stateTime.milliseconds());
    }
}
