package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends BodyPart {
    // Arm speeds
    private static final double MIN_ARM_SPEED = -0.9;
    private static final double MAX_ARM_SPEED = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.2;
    private static final double NO_POWER = 0.0;

    // Min and max pos of the arm
    public static final int MIN_POS = 0;
    public static final int MAX_POS = 2000;

    // Vars for the arm motors
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;

    // Var for the shoulder
    private Shoulder shoulder;
    private boolean hold = false;

    /**
     * Constructor for the arm
     * @param hardwareMap map with arm parts
     * @param gamepad the gamepad used to control the arm
     */
    public Arm(HardwareMap hardwareMap, Gamepad gamepad, Shoulder shoulder) {
        // Assignments
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor;
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor;
        this.gamepad = gamepad;
        this.shoulder = shoulder;

        // Setup
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setTargetPosition(0);
        armMotorRight.setTargetPosition(0);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Method adds important things to telemetry
     * @param telemetry
     */
    public void debugTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Arm Counts Left", armMotorLeft.getCurrentPosition());
        telemetry.addData("Arm Counts Right", armMotorRight.getCurrentPosition());
        telemetry.addData("Arm Ratio", getArmRatio());
    }

    @Override
    public int getCurrentPosition() {
        return armMotorLeft.getCurrentPosition();
    }

    /**
     * Sets the value of the shoulder for initialization
     * @param shoulder the shoulder object used
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
        setPosition(power, MIN_POS);
    }

    public void gotoMax(double power)
    {
        setPosition(power, MAX_POS);
    }

    /**
     * Completely de-power the arm motors (gravity takes over)
     */
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
    public void setPosition(double power, int position) {
        // Ensure inputs are valid
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        position = Range.clip(position, MIN_POS, MAX_POS);

        // Set new position and power the motors
        armMotorLeft.setTargetPosition(position);
        armMotorRight.setTargetPosition(position);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);

        // Generate a new motor protection thread
        protectMotors(position);

        // Tell the shoulder to update for ground protection
        if (shoulder != null) shoulder.targetArmRatio((double)(position - MIN_POS) / (double)(MAX_POS - MIN_POS));
    }

    /**
     * This sets hold externally (forces a new hold if false)
     * @param hold if true we implement a hold call
     */
    public void setHold(boolean hold) {
        this.hold = hold;
    }

    @Override
    public void safeHold(int position)
    {
        // We've noticed the motors fighting themselves while holding (rigid tool end).
        // Two ways to fix this I think:
        //   1) Allow the tool to rotate slightly for differences in motor speeds and ticks
        //   2) Power one motor for hold and test
        int posLeft = Range.clip(armMotorLeft.getCurrentPosition(), MIN_POS, MAX_POS);
        int posRight = Range.clip(armMotorRight.getCurrentPosition(), MIN_POS, MAX_POS);
        if (posLeft < posRight) {
            armMotorRight.setTargetPosition(posLeft);
            armMotorLeft.setTargetPosition(posLeft);
            armMotorRight.setPower(NO_POWER);
            armMotorLeft.setPower(HOLD_POWER);
        } else {
            armMotorRight.setTargetPosition(posRight);
            armMotorLeft.setTargetPosition(posRight);
            armMotorRight.setPower(HOLD_POWER);
            armMotorLeft.setPower(NO_POWER);
        }

        // This is belt slip protection
        if(position <= MIN_POS)
        {
            // TODO - If we are at ARM_IN and the motor ticks don't match, one of them slipped
            // so we should reset the encoder counts here to reset for full extension
        }

        // I've stopped moving, tell the shoulder to recheck safety one last time
        if (shoulder != null) shoulder.setHold(false);

        // Cancel any pending safeHolds
        protectionThread.interrupt();
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            if(!ignoreGamepad)
            {
                // Sets the arm speed to a number MIN to MAX based on the left stick's position
                double power = gamepad.left_stick_y;
                if (!hold && Math.abs(power) <= TRIM_POWER) {
                    safeHold(armMotorLeft.getCurrentPosition());
                    hold = true;
                } else if (power < -TRIM_POWER) {
                    // Calling setArmPosition here adds the motor protection, even if the driver
                    // holds the retraction stick down forever
                    setPosition(Math.abs(power), MIN_POS);
                    hold = false;
                } else if (power > TRIM_POWER) {
                    // Calling setArmPosition here adds the motor protection, even if the driver
                    // holds the extension stick up forever
                    setPosition(power, MAX_POS);
                    hold = false;
                }
            }

            // Short sleep to keep this loop from saturating
            try {
                sleep(LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }
}
