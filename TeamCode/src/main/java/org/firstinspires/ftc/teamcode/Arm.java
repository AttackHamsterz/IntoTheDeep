package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm extends Thread{
    //min and max speed of the arm  🦾🚗💨
    private static final double MIN_ARM_SPEED = -1;
    private static final double MAX_ARM_SPEED = -1;

    //min and max rotation/position(pos) of the arm 🔄
    private static final int MIN_POS = 0;
    private static final int MAX_POS = 1000;

    //variables (vars) for the arm motors  🦾🐝
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;

    //vars for the shoulder and gamepad for the constructor 🎮🛠️
    //private Shoulder shoulder;
    private final Gamepad gamepad;

    //vars for keeping track of arm position  🦾📍
    private int totalCountsLeft;
    private int totalCountsRight;

    /**
     * Constroctor for the arm!!! 🦾🛠️
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
}
