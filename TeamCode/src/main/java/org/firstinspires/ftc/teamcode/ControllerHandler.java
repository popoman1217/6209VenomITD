package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerHandler {

    OpMode master;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    public void initController(OpMode opMode)
    {
        master = opMode;

        currentGamepad1.copy(master.gamepad1);
        currentGamepad2.copy(master.gamepad2);
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }

    public boolean isGP1APressed() {
        return master.gamepad1.a;
    }

    public boolean isGP1BPressed() {
        return master.gamepad1.b;
    }

    public boolean isGP1XPressed() {
        return master.gamepad1.x;
    }

    public boolean isGP1YPressed() {
        return master.gamepad1.y;
    }

    public boolean isGP1DpadUpPressed() {
        return master.gamepad1.dpad_up;
    }

    public boolean isGP1DpadDownPressed() {
        return master.gamepad1.dpad_down;
    }

    public boolean isGP1DpadLeftPressed() {
        return master.gamepad1.dpad_left;
    }

    public boolean isGP1DpadRightPressed() {
        return master.gamepad1.dpad_right;
    }

    public boolean isGP1LeftBumperPressed() {
        return master.gamepad1.left_bumper;
    }

    public boolean isGP1RightBumperPressed() {
        return master.gamepad1.right_bumper;
    }

    public boolean isGP1StartPressed() {
        return master.gamepad1.start;
    }

    public boolean isGP1BackPressed() {
        return master.gamepad1.back;
    }

    public boolean isGP2APressed() {
        return master.gamepad2.a;
    }

    public boolean isGP2BPressed() {
        return master.gamepad2.b;
    }

    public boolean isGP2XPressed() {
        return master.gamepad2.x;
    }

    public boolean isGP2YPressed() {
        return master.gamepad2.y;
    }

    public boolean isGP2DpadUpPressed() {
        return master.gamepad2.dpad_up;
    }

    public boolean isGP2DpadDownPressed() {
        return master.gamepad2.dpad_down;
    }

    public boolean isGP2DpadLeftPressed() {
        return master.gamepad2.dpad_left;
    }

    public boolean isGP2DpadRightPressed() {
        return master.gamepad2.dpad_right;
    }

    public boolean isGP2LeftBumperPressed() {
        return master.gamepad2.left_bumper;
    }

    public boolean isGP2RightBumperPressed() {
        return master.gamepad2.right_bumper;
    }

    public boolean isGP2StartPressed() {
        return master.gamepad2.start;
    }

    public boolean isGP2BackPressed() {
        return master.gamepad2.back;
    }

    public double getGP1LeftStickX() {
        return master.gamepad1.left_stick_x;
    }

    public double getGP1LeftStickY() {
        return master.gamepad1.left_stick_y;
    }

    public double getGP1RightStickX() {
        return master.gamepad1.right_stick_x;
    }

    public double getGP1RightStickY() {
        return master.gamepad1.right_stick_y;
    }

    public double getGP1LeftTrigger() {
        return master.gamepad1.left_trigger;
    }

    public double getGP1RightTrigger() {
        return master.gamepad1.right_trigger;
    }

    public double getGP2LeftStickX() {
        return master.gamepad2.left_stick_x;
    }

    public double getGP2LeftStickY() {
        return master.gamepad2.left_stick_y;
    }

    public double getGP2RightStickX() {
        return master.gamepad2.right_stick_x;
    }

    public double getGP2RightStickY() {
        return master.gamepad2.right_stick_y;
    }

    public double getGP2LeftTrigger() {
        return master.gamepad2.left_trigger;
    }

    public double getGP2RightTrigger() {
        return master.gamepad2.right_trigger;
    }

    public boolean isGP1APressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.a && !previousGamepad1.a;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1BPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.b && !previousGamepad1.b;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1XPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.x && !previousGamepad1.x;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1YPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.y && !previousGamepad1.y;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1DpadUpPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.dpad_up && !previousGamepad1.dpad_up;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1DpadDownPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.dpad_down && !previousGamepad1.dpad_down;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1DpadLeftPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.dpad_left && !previousGamepad1.dpad_left;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1DpadRightPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.dpad_right && !previousGamepad1.dpad_right;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1LeftBumperPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.left_bumper && !previousGamepad1.left_bumper;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1RightBumperPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1StartPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.start && !previousGamepad1.start;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP1BackPressed1Frame() {
        currentGamepad1.copy(master.gamepad1);
        boolean result = currentGamepad1.back && !previousGamepad1.back;
        previousGamepad1.copy(currentGamepad1);
        return result;
    }

    public boolean isGP2APressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.a && !previousGamepad2.a;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2BPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.b && !previousGamepad2.b;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2XPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.x && !previousGamepad2.x;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2YPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.y && !previousGamepad2.y;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2DpadUpPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.dpad_up && !previousGamepad2.dpad_up;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2DpadDownPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.dpad_down && !previousGamepad2.dpad_down;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2DpadLeftPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.dpad_left && !previousGamepad2.dpad_left;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2DpadRightPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.dpad_right && !previousGamepad2.dpad_right;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2LeftBumperPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2RightBumperPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.right_bumper && !previousGamepad2.right_bumper;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2StartPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.start && !previousGamepad2.start;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

    public boolean isGP2BackPressed1Frame() {
        currentGamepad2.copy(master.gamepad2);
        boolean result = currentGamepad2.back && !previousGamepad2.back;
        previousGamepad2.copy(currentGamepad2);
        return result;
    }

}
