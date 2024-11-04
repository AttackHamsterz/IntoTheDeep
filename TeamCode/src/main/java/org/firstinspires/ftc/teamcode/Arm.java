package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    // Min and max speed of the arm
    private static final double MIN_ARM_SPEED = -0.9;
    private static final double MAX_ARM_SPEED = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.4;

    // Min and max pos of the arm
    public static final int MIN_POS = 0;
    public static final int MAX_POS = 1900;

    // Vars for the arm motors
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;

    // Var for the shoulder and gamepad for the constructor
    private Shoulder shoulder;
    private final Gamepad gamepad;

    private boolean ignoreGamepad = false;
    private boolean hold = false;

    public int getArmPosition() {
       return Math.round((float) (armMotorLeft.getCurrentPosition() + armMotorRight.getCurrentPosition()) / 2);
    }
    /**
     * Allows a user to toggle using the gamepad
     * @param ignoreGamepad true to ingonre gamepad inputs
     */
    public void ignoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }

    /**
     * Constructor for the arm
     * @param armMotorLeft the motor for the first arm
     * @param armMotorRight the motor for the second arm
     * @param gamepad the gamepad used to control the arm
     */
    public Arm(DcMotor armMotorLeft, DcMotor armMotorRight, Gamepad gamepad, Shoulder shoulder) {
        this.armMotorLeft = armMotorLeft;
        this.armMotorRight = armMotorRight;
        this.gamepad = gamepad;
        this.shoulder = shoulder;
        this.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the value of the shoulder for initialization
     * @param shoulder the shoulder motor used
     */
    public void setShoulder(Shoulder shoulder) {
        this.shoulder = shoulder;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio()
    {
        return Range.clip((double) (armMotorLeft.getCurrentPosition() - MIN_POS) / (double) (MAX_POS - MIN_POS), 0.0, 1.0);
    }

    public void gotoMin(double power)
    {
        setArmPosition( power, MIN_POS);
    }

    public void gotoMax(double power)
    {
        setArmPosition(power, MAX_POS);
    }

    public void halt()
    {
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
    }

    /**
     * Sets the position of the arm
     * @param power the power of the arm's movements
     * @param position arm position
     */
    public void setArmPosition(double power, int position) {
        // Ensure inputs are valid
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        position = Range.clip(position, MIN_POS, MAX_POS);

        // Set new position and power the motors
        armMotorLeft.setTargetPosition(position);
        armMotorRight.setTargetPosition(position);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);

        // Tell the shoulder to update - TODO this should just tell it the new MIN_POS if necessary
        if (shoulder != null) shoulder.setHold(false);
    }

    /**
     * Sets the position of the arm based on a value 0.0-1.0
     * @param power the power of the arm
     * @param position a value 0.0-1.0 which determines the position
     */
    public void setArmPosition(double power, double position) {
        // Make sure inputs are valid
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        position = Range.clip(position, 0.0, 1.0);

        // Calculate new position, set it and power the motors
        int pos = Range.clip((int)Math.round(position * (MAX_POS - MIN_POS)) + MIN_POS, MIN_POS, MAX_POS);
        armMotorLeft.setTargetPosition(pos);
        armMotorRight.setTargetPosition(pos);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);

        // Tell the shoulder to update - TODO this should just tell it the new MIN_POS if necessary
        if (shoulder != null) shoulder.setHold(false);
    }

    /**
     * This sets hold externally (forces a new hold value if false)
     * @param hold
     */
    public void setHold(boolean hold) {
        this.hold = hold;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            if(!ignoreGamepad)
            {
                // Sets the arm speed to a number -1 through 1 based on the left stick's position
                double power = gamepad.left_stick_y;
                if (!hold && Math.abs(power) <= TRIM_POWER) {
                    int pos = Range.clip(armMotorLeft.getCurrentPosition(),MIN_POS,MAX_POS);
                    armMotorLeft.setTargetPosition(pos);
                    armMotorRight.setTargetPosition(pos);
                    armMotorLeft.setPower(HOLD_POWER);
                    armMotorRight.setPower(HOLD_POWER);
                    hold = true;
                    if (shoulder != null) shoulder.setHold(false);
                } else if (power < -TRIM_POWER) {
                    armMotorLeft.setTargetPosition(MIN_POS);
                    armMotorRight.setTargetPosition(MIN_POS);
                    armMotorLeft.setPower(Math.abs(power));
                    armMotorRight.setPower(Math.abs(power));
                    hold = false;
                    if (shoulder != null) shoulder.setHold(false);
                } else if (power > TRIM_POWER) {
                    armMotorLeft.setTargetPosition(MAX_POS);
                    armMotorRight.setTargetPosition(MAX_POS);
                    armMotorLeft.setPower(power);
                    armMotorRight.setPower(power);
                    hold = false;
                    if (shoulder != null) shoulder.setHold(false);
                }
            }

            // Short sleep to keep this loop from saturating
            try {
                sleep(50);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }
}
