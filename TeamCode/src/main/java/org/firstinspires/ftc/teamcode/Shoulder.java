package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    //Variables for shoulder speed
    private static final double MIN_SHOULDER_SPEED = -0.5;
    private static final double MAX_SHOULDER_SPEED = 1;

    //Setting up variables for min and max pos
    public int MIN_POS;
    public int MAX_POS;

    //Pre-set min and max pos based on if the arm is in or out
    public static int MIN_POS_ARM_IN = 0;
    public static int MAX_POS_ARM_IN = -1400;
    // change later
    public static int MIN_POS_ARM_OUT = -335;
    public static int MAX_POS_ARM_OUT = -1400;

    public static int HANG_HEIGHT_LOWER;
    public static int HANG_HEIGHT_UPPER;

    //Bucket Heights
    public static int LOWER_BUCKET = -844;
    public static int UPPER_BUCKET = -1242;

    //Other Heights
    public static int SEARCH_HEIGHT = -187;

    //Amount arm will move for manual adjustments
    //public static int SHOULDER_MANUAL = 100;

    //Setting up vars of threading
    private final DcMotor shoulderMotor;
    private final Arm arm;
    private final Gamepad gamepad;
    //Var for motor counts
    private int totalCounts;
    private boolean ignoreGamepad = false;
    private boolean isMoving = false;
    private int targetPos = 0;
    private boolean hold = false;

    public void ignoreGamepad () {
        ignoreGamepad = true;
    }


    /**
     * Constructor for the shoulder
     *
     * @param shoulderMotor the motor for the shoulder
     * @param gamepad       the gamepad used for controlling the shoulder
     */
    //Add in arm later
    public Shoulder(DcMotor shoulderMotor, Arm arm, Gamepad gamepad) {
        this.shoulderMotor = shoulderMotor;
        this.arm = arm;
        this.gamepad = gamepad;
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Function to get motor counts
     *
     * @return total counts
     */
    protected int getShoulderCounts() {
        return totalCounts;
    }

    public void setHold(boolean hold) {
        this.hold = hold;
    }

    /**
     * Gets the ratio of shoulder base off its position
     *
     * @return double between 0.0-1.0
     */

    public double getShoulderRatio() {
        //Sets a position variable of the robot's current position
        double pos = shoulderMotor.getCurrentPosition();
        //Checks to see if the arm is between 0 and -225
        if (pos > MIN_POS_ARM_OUT) {
            //Returns a double between 0.0-1.0 based on where the shoulder is between 0 and -225
            return Range.clip(((double) pos - MIN_POS_ARM_IN) / ((double) MIN_POS_ARM_OUT - (double) MIN_POS_ARM_IN), 0.0, 1.0);
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
     *
     * @param power    double, power of the shoulder motor
     * @param position int, position/angle to set the shoulder to
     */
    public void setPosition(double power, int position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder
        shoulderMotor.setTargetPosition(Range.clip(position, MIN_POS_ARM_IN, MIN_POS_ARM_IN));
        shoulderMotor.setPower(power);
    }

    public  void  setShoulderPosition(double power, int position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setPower(power);
    }



    @Override
    public void run() {
        while (!isInterrupted()) {
            //Sets the min pos to an int value based on how far the arm is out
            //MIN_POS = (int) Math.round(arm.getArmRatio() * (double)(MIN_POS_ARM_OUT-MIN_POS_ARM_IN)) + SEARCH_HEIGHT;
            //Sets the max pos to an int value based on how far the arm is out
            //MAX_POS = MAX_POS_ARM_OUT;

            float power = gamepad.right_stick_y;

            if (!hold && Math.abs(power) < 0.15) {
                shoulderMotor.setPower(0.9);
                shoulderMotor.setTargetPosition(Range.clip(shoulderMotor.getCurrentPosition(), MAX_POS_ARM_IN, MIN_POS_ARM_IN));
                hold = true;
            } else if (power < -0.15) {
                shoulderMotor.setPower(Math.abs(power));
                shoulderMotor.setTargetPosition(MAX_POS_ARM_IN);
                hold = false;
                totalCounts = shoulderMotor.getCurrentPosition();
            } else if (power > 0.15) {
                shoulderMotor.setPower(power);
                shoulderMotor.setTargetPosition(MIN_POS_ARM_IN);
                hold = false;
                totalCounts = shoulderMotor.getCurrentPosition();
            }

        }

    }
}
