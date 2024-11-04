package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    // Arm speeds
    private static final double MIN_ARM_SPEED = -0.9;
    private static final double MAX_ARM_SPEED = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.4;
    private static final double NO_POWER = 0.0;

    // Motor overload protection
    private static final long MOTOR_CHECK_PERIOD_MS = 250;  // Check 4 times a second
    private static final int CLOSE_ENOUGH_TICKS = 10; // Turn off the other motor when we are close
    private Thread protectionThread = new Thread();

    // Min and max pos of the arm
    public static final int MIN_POS = 0;
    public static final int MAX_POS = 1950;

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
    public void setArmPosition(double power, int position) {
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
        // TODO this should just tell shoulder the new target position
        if (shoulder != null) shoulder.setHold(false);
    }

    /**
     * This sets hold externally (forces a new hold value if false)
     * @param hold
     */
    public void setHold(boolean hold) {
        this.hold = hold;
    }

    /**
     * This sets up motor protection to avoid overloading the motors.  It needs the distance
     */
    private void protectMotors(int targetPosition)
    {
        // Cancel old thread
        protectionThread.interrupt();

        // Start a thread that performs safe hold when the time is right
        targetPosition = Range.clip(targetPosition, MIN_POS, MAX_POS);
        Integer position = new Integer(targetPosition);
        protectionThread = new Thread(() -> {
            try{
                do {
                    sleep(MOTOR_CHECK_PERIOD_MS);
                }while(Math.abs(armMotorLeft.getCurrentPosition()-position) > CLOSE_ENOUGH_TICKS);
                safeHold();
            } catch (InterruptedException e) {
            }
        });
        protectionThread.start();
    }

    /**
     * We've noticed the motors fighting themselves while holding (rigid tool end).
     * Two ways to fix this I think:
     *   1) Allow the tool to rotate slightly for differences in motor speeds and ticks
     *   2) Power one motor for hold and test
     */
    public void safeHold()
    {
        int posLeft = Range.clip(armMotorLeft.getCurrentPosition(), MIN_POS, MAX_POS);
        armMotorRight.setTargetPosition(posLeft);
        armMotorLeft.setTargetPosition(posLeft);
        armMotorRight.setPower(NO_POWER);
        armMotorLeft.setPower(HOLD_POWER);

        // Cancel any future safeHolds
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
                    safeHold();
                    hold = true;
                    if (shoulder != null) shoulder.setHold(false);
                } else if (power < -TRIM_POWER) {
                    // Calling setArmPosition here adds the motor protection, even if the driver
                    // holds the retraction stick down forever
                    setArmPosition(Math.abs(power), MIN_POS);
                    hold = false;
                    if (shoulder != null) shoulder.setHold(false);
                } else if (power > TRIM_POWER) {
                    // Calling setArmPosition here adds the motor protection, even if the driver
                    // holds the extension stick up forever
                    setArmPosition(Math.abs(power), MAX_POS);
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
