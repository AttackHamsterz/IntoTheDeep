package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    //Min and max speed of the arm
    private static final double MIN_ARM_SPEED = -1;
    private static final double MAX_ARM_SPEED = 1;

    //Min and max pos of the arm
    private static final int MIN_POS = 0;
    private static final int MAX_POS = 2780;

    //Vars for the arm motors
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;
    //Var for the shoulder and gamepad for the constructor
    private Shoulder shoulder;
    private final Gamepad gamepad;
    //Vars for keeping track of the arm's position
    private int totalCountsLeft;
    private int totalCountsRight;

    private boolean ignoreGamepad = false;

    public void ignoreGamepad() {ignoreGamepad = true;}

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
     * Gets the position of the first arm motor
     * @return totalCounts1, the position of the first arm motor
     */
    protected int getArmCountsLeft() {
        return totalCountsLeft;
    }

    /**
     * Gets the position of the second arm motor
     * @return totalCounts2, the position of the second arm motor
     */
    protected int getArmCountsRight() {
        return totalCountsRight;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio() {return Range.clip((double) totalCountsLeft / (double) MAX_POS, 0.0, 1.0);}

    /**
     * Sets the position of the arm
     * @param power the power of the arm
     * @param position a value 0-2180 that sets the arm position
     */
    public void setArmPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        armMotorLeft.setTargetPosition(position);
        armMotorRight.setTargetPosition(position);
    }

    /**
     * Sets the position of the arm based on a value 0.0-1.0
     * @param power the power of the arm
     * @param position a value 0.0-1.0 which determines the position
     */
    public void setArmPosition(double power, double position) {
        //Makes sure the power is between 0-1
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        //Sets the power of both arm motors
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        //Sets the position of both arms based on the position given
        //Multiplies the range of the arm movement by the position and adds the min pos (in case min pos isn't zero)
        armMotorLeft.setTargetPosition((int) ((MAX_POS-MIN_POS) * position) + MIN_POS);
        armMotorRight.setTargetPosition((int) ((MAX_POS-MIN_POS) * position) + MIN_POS);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            //Gets the current position of the arm
            totalCountsLeft = armMotorLeft.getCurrentPosition();
            totalCountsRight = armMotorRight.getCurrentPosition();

            //Sets the arm speed to a number -1 through 1 based on the left stick's position
            double ARM_SPEED = (ignoreGamepad) ? 0: gamepad.left_stick_y;
            //If the arm speed is positive, then we move the arm towards the min pos
            //If the arm speed is negative, then we move the arm towards the max pos
            int pos = (ARM_SPEED >= 0) ? MIN_POS:MAX_POS;
            //Sets the arm pos based on the pos we just calculated
            setArmPosition(ARM_SPEED, pos);

            if (shoulder != null) {
                shoulder.setHold(false);
                //Gets the shoulder's current position
                /*
                int shoulderCounts = shoulder.getShoulderCounts();
                //Checks if the shoulder is close to the floor
                if (shoulderCounts > Shoulder.MIN_POS_ARM_OUT) {
                    //Finds the ratio of the shoulder in the floor zone
                    //Divides shoulder's current position int the zone by the full range of the floor zone
                    double shoulderRatio = (double) (shoulderCounts - Shoulder.MIN_POS_ARM_IN) / (double) (Shoulder.MIN_POS_ARM_OUT-Shoulder.MIN_POS_ARM_IN);
                    //Gets the arm ratio, which is how far the arm is extended
                    double armRatio = getArmRatio();
                    //Checks if the arm is too far extended
                    if (armRatio > shoulderRatio) {
                        //Multiplies the full range of the floor zone by the arm ratio and adds the min pos (in case the min pos wasn't 0)
                        //Places the shoulder in the floor zone based on the arm ratio
                        int newPos = (int) ((double) (Shoulder.MIN_POS_ARM_OUT-Shoulder.MIN_POS_ARM_IN) * armRatio) + Shoulder.MIN_POS_ARM_IN;
                        shoulder.setShoulderPosition(0.75, newPos);
                    }
                }

                 */
            }


        }
    }
}
