package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Tool extends Thread{
    private Servo clawServoLeft;
    private Servo clawServoRight;
    private Servo wristServo;
    //Vars for the gamepad and shoulder
    private Gamepad gamepad;
    //Vars for the positions of each claw servo
    private double totalCountsLeft;
    private double totalCountsRight;
    private double totalCountsWrist;
    private boolean ignoreGamepad = false;
    // open and close positions
    // wrist rotations

    public void ignoreGamepad() {ignoreGamepad = true;}

    public Tool(Servo clawServoLeft, Servo clawServoRight, Servo wristServo, Gamepad gamepad) {
        this.clawServoLeft = clawServoLeft;
        this.clawServoRight = clawServoRight;
        this.wristServo = wristServo;
        this.gamepad = gamepad;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            if (!ignoreGamepad) {
                if (gamepad.left_bumper) {
                    //Closes the left claw
                    clawServoLeft.setPosition(clawServoLeft.getPosition()+0.01);
                    //If the left trigger is pressed
                } else if (gamepad.left_trigger > 0) {
                    //Opens the left claw

                }

                //If the right bumper is pressed
                if (gamepad.right_bumper) {
                    //Closes the right claw
                    clawServoRight.setPosition(clawServoRight.getPosition()+0.01);
                    //If the right trigger is pressed
                } else if (gamepad.right_trigger > 0) {
                    //Opens the right claw

                }
            }
        }
    }

}
