package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shoulder extends Thread {
    //variables for shoulder speed
    private static final double MIN_SHOULDER_SPEED = -0.5;
    private static final double MAX_SHOULDER_SPEED = 0.5;

    //setting up variables for min and max pos
    private int MIN_POS;
    private int MAX_POS;

    //setting up vars of threading
    private final DcMotor shoulderMotor;
    //private final Arm arm;
    private final Gamepad gamepad;

    //var for motor counts
    private int totalCounts;
    private boolean isMoving = false;
    private boolean ignoreGamepad = false;
    private int targetPos = 0;
    private int threshold = 20;

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
   public void ignoreGamepad () {
        ignoreGamepad = true;
    }
}
