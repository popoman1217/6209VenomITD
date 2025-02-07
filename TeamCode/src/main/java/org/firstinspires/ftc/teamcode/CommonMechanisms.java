package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class CommonMechanisms {
    OpMode master;
    StateHandler stateHandler;
    Mechanisms mechanisms;
    int tarPose = 2300;
    public void init(OpMode opMode, StateHandler sh, Mechanisms m)
    {
        master = opMode;
        stateHandler = sh;
        mechanisms = m;
    }


    public void moveFromClampedToPlaceAndBack(String nextState)
    {
        stateHandler.switchState("beginning");
        if (stateHandler.state.equals("beginning") )
        {
            mechanisms.outTakePivotLeft.setPosition(Mechanisms.LOW_OT_ARM_POSL);
            mechanisms.outTakePivotRight.setPosition(Mechanisms.LOW_OT_ARM_POSR);
            stateHandler.stateUpdate();
            stateHandler.switchState("raising lift", 250, mechanisms, StateHandler.CoRoutines.None);
        }


        if (stateHandler.state.equals("raising lift"))
        {
            mechanisms.setMacroBrakeValsOT();
            mechanisms.resetMacroVals(false);
            mechanisms.moveOTLiftEncoder(.7, tarPose, 10000);
            stateHandler.stateUpdate();
            stateHandler.switchState("moving arm", 500, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        if (stateHandler.state.equals("moving arm"))
        {
            mechanisms.update();
            mechanisms.powerOTPIDToTarget();
            mechanisms.outTakePivotLeft.setPosition(Mechanisms.HIGH_OT_ARM_POSL);
            mechanisms.outTakePivotRight.setPosition(Mechanisms.HIGH_OT_ARM_POSR);
            stateHandler.stateUpdate();
            stateHandler.switchState("opening claw", 500, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        if (stateHandler.state.equals("opening claw"))
        {
            mechanisms.update();
            mechanisms.powerOTPIDToTarget();
            mechanisms.outTakeClaw.setPosition(mechanisms.OPEN_CLAW_POS);
            stateHandler.stateUpdate();
            stateHandler.switchState("lowering arm", 300, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        if (stateHandler.state.equals("lowering arm"))
        {
            mechanisms.update();
            mechanisms.powerOTPIDToTarget();
            mechanisms.outTakePivotLeft.setPosition(Mechanisms.LOW_OT_ARM_POSL);
            mechanisms.outTakePivotRight.setPosition(Mechanisms.LOW_OT_ARM_POSR);
            stateHandler.stateUpdate();
            stateHandler.switchState("lowering lift", 300, mechanisms, StateHandler.CoRoutines.OTPID);
        }

        // mechanisms.transferMacroAuto();

        if (stateHandler.state.equals("lowering lift"))
        {
            mechanisms.moveOTLiftEncoder(.7, -tarPose, 2500);
            //mechanisms.setOTLiftPower(0);
            stateHandler.stateUpdate();
            stateHandler.switchState(nextState, 500, mechanisms, StateHandler.CoRoutines.OTPID);
        }
    }
}
