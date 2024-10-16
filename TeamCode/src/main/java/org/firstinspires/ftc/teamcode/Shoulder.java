package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    //Variables for shoulder speed
    private static final double MIN_SHOULDER_SPEED = -0.5;
    private static final double MAX_SHOULDER_SPEED = 1;

    //Setting up variables for min and max pos
    private int MIN_POS;
    private int MAX_POS;

    //Pre-set min and max pos based on if the arm is in or out
    public static int MIN_POS_ARM_IN = -20;
    public static int MAX_POS_ARM_IN = -2657;
    public static int MIN_POS_ARM_OUT = -225;
    public static int MAX_POS_ARM_OUT = -2417;

    //Amount arm will move for manual adjustments
    public static int SHOULDER_MANUAL = 100;

    //Setting up vars of threading
    private final DcMotor shoulderMotor;
    //private final Arm arm;
    private final Gamepad gamepad;
    //Var for motor counts
    private int totalCounts;
    private boolean ignoreGamepad = false;
    private boolean isMoving = false;
    private int targetPos = 0;

    public void ignoreGamepad () {
        ignoreGamepad = true;
    }

    /**
     * Constructor for the shoulder
     * @param shoulderMotor the motor for the shoulder
     * @param gamepad the gamepad used for controlling the shoulder
     */
    //Add in arm later
    public Shoulder(DcMotor shoulderMotor, Gamepad gamepad){
        this.shoulderMotor = shoulderMotor;
        //this.arm = arm;
        this.gamepad = gamepad;
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Function to get motor counts
     * @return total counts
     */
    protected int getShoulderCounts() {return totalCounts;}

    /**
     * Gets the ratio of shoulder base off its position
     * @return double between 0.0-1.0
     */

    public double getShoulderRatio() {
        //Sets a position variable of the robot's current position
        double pos = shoulderMotor.getCurrentPosition();
        //Checks to see if the arm is between 0 and -225
        if (pos > MIN_POS_ARM_OUT) {
            //Returns a double between 0.0-1.0 based on where the shoulder is between 0 and -225
            return Range.clip(((double) pos - MIN_POS_ARM_IN) / ((double) MIN_POS_ARM_OUT - (double)MIN_POS_ARM_IN), 0.0, 1.0);
            //Checks to see if the arm is between -2417 and -2657
        } else if (pos < MAX_POS_ARM_OUT) {
            //Returns a double between 0.0-1.0 based on where the shoulder is between -2417 and -2657
            return Range.clip(((double) pos - MAX_POS_ARM_IN) / ((double) MAX_POS_ARM_OUT - (double) MAX_POS_ARM_IN), 0.0, 1.0);
        } else {
            //Returns 1.0 if the arm is outside the prior two ranges
            return 1.0;
        }
    }

    public double shoulderAngle() {
        return (double) shoulderMotor.getCurrentPosition() / MAX_POS_ARM_IN;
    }



    /**
     * Sets the position of the shoulder
     * @param power double, power of the shoulder motor
     * @param position int, position/angle to set the shoulder to
     */
    public  void  setShoulderPosition(double power, int position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setPower(power);
    }

    /**
     * Sets the position of the shoulder based on a value 0.0-1.0
     * @param power the power of the shoulder
     * @param position a value 0.0-1.0 that sets the position of the shoulder
     */
    public  void setShoulderPosition(double power, double position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder based on the position given
        //Multiplies the range of the shoulder movement by the position and adds the min pos
        shoulderMotor.setTargetPosition((int) ((MAX_POS_ARM_OUT-MIN_POS_ARM_IN) * position) + MIN_POS_ARM_IN);
        shoulderMotor.setPower(power);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            //Sets total counts to the shoulder's current position
            totalCounts = shoulderMotor.getCurrentPosition();
/*
            //Sets the min pos to an int value based on how far the arm is out
            MIN_POS = (int) Math.round(arm.getArmRatio() * (MIN_POS_ARM_OUT-MIN_POS_ARM_IN) + MIN_POS_ARM_IN);
            //Sets the max pos to an int value based on how far the arm is out
            MAX_POS = (int) Math.round(arm.getArmRatio() * (MAX_POS_ARM_OUT-MAX_POS_ARM_IN) + MAX_POS_ARM_IN);

 */


                if (gamepad.dpad_left) {
                    int pos = shoulderMotor.getCurrentPosition() + SHOULDER_MANUAL;
                    setShoulderPosition(0.2, Range.clip(pos, MAX_POS, MIN_POS));
                    //Lower manually
                } else if (gamepad.dpad_right) {
                    int pos = shoulderMotor.getCurrentPosition() -SHOULDER_MANUAL;
                    setShoulderPosition(0.2, Range.clip(pos, MAX_POS, MIN_POS));
                    //Raise manually
                    }



            //Doesn't work since shoulder de-powers if joystick reaches neutral position
/*
            double SHOULDER_SPEED = gamepad.right_stick_y;
            int pos;
            double power;
            if (SHOULDER_SPEED > -0.15 && SHOULDER_SPEED < 0.15) {
                pos = totalCounts;
                power = 1;
                shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (SHOULDER_SPEED > 0) {
                pos = MIN_POS;
                power = SHOULDER_SPEED;

            } else {
                pos = MAX_POS;
                power = Math.abs(SHOULDER_SPEED);

            }
            setPosition(power, pos);

 */
/*
            if (gamepad.a) {
                //setPosition(SHOULDER_SPEED, MIN_POS);
                //On floor front
            } else if (gamepad.b) {
                //Drive position
            } else if (gamepad.y) {
                //Between line 1 and 2 front
            } else if (gamepad.x) {
                //Between line 2 and 3 front
            } else if (gamepad.dpad_down) {
                //On floor back
            } else if (gamepad.dpad_up) {
                //Between line 1 and 2 back
            } else if (gamepad.dpad_left) {
                //Between line 2 and 3 back
            } else if (gamepad.dpad_up) {
                //Straight up and down
            }

 */
        }
    }

}
