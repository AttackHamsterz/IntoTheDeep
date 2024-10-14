package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    //min and max speed of the arm  🦾🚗💨
    private static final double MIN_ARM_SPEED = -1;
    private static final double MAX_ARM_SPEED = -1;

    //min and max rotation/position(pos) of the arm 🔄
    private static final int MIN_POS = 0;
    private static final int MAX_POS = 1000;
    private static int SHOULDER_MAX;

    //variables (vars) for the arm motors  🦾🐝
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;

    //vars for the shoulder and gamepad for the constructor 🎮🛠️
    private Shoulder shoulder;
    private final Gamepad gamepad;

    //vars for keeping track of arm position  🦾📍
    private int totalCountsLeft;
    private int totalCountsRight;

    /**
     * Constructor for the arm!!! 🦾🛠️
     * @param armMotorLeft
     * @param armMotorRight
     * @param gamepad
     */
    public Arm(DcMotor armMotorLeft, DcMotor armMotorRight, Gamepad gamepad) {
        this.armMotorLeft = armMotorLeft;
        this.armMotorRight = armMotorRight;
        this.gamepad = gamepad;
        this.armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setShoulder(Shoulder shoulder) {
        this.shoulder = shoulder;
    }

    protected int getArmCounts1() {
        return totalCountsLeft;
    }

    protected int getArmCounts2() {
        return totalCountsRight;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio() {return Range.clip((double) totalCountsLeft / (double) MAX_POS, 0.0, 1.0);}

    private void setPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        armMotorRight.setTargetPosition(position);
        armMotorLeft.setTargetPosition(position);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            if (shoulder != null) {
                SHOULDER_MAX = (int) Math.round(shoulder.getShoulderRatio());
            } else {
                SHOULDER_MAX = 0;
            }

            totalCountsLeft = armMotorLeft.getCurrentPosition();
            totalCountsRight = armMotorRight.getCurrentPosition();

            double ARM_SPEED = gamepad.left_stick_y;
            int pos = (ARM_SPEED >= 0) ? MIN_POS:MAX_POS;
            setPosition(ARM_SPEED, pos);

        }
    }


}
