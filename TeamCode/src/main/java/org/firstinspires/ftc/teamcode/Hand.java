package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hand extends Thread{
    private static final double MIN_POS = 0.0; // -90 degrees
    private static final double MAX_POS = 1.0; // +90 degrees
    private static final double CTR_POS = 0.5; // 0 degrees

    private static final double MIN_POWER = 0.0;  // Full reverse power
    private static final double MAX_POWER = 1.0;  // Full forward power
    private static final double GRAB_POWER = 1.0;
    private static final double SPIN_POWER = 0.8; // Amount to add or remove from no-power
    private static final double NO_POWER = 0.0; // Continous servo stop
    private static final double RELEASE_POWER = 0.5;

    private static final double MIN_TRIGGER = 0.1;
    private static final double MANUAL_SPIN_INCREMENT = 0.05;
    private static final long INTAKE_MS = 100;
    private static final long RELEASE_MS = 500;
    private static final long SPIN_MS = 400;

    private CRServo left;
    private CRServo right;
    private Servo wrist;
    private boolean ignoreGamepad;
    private final Gamepad gamepad;
    private Thread stopThread;

    /**
     * Construct the hand with 3 servos and a gamepad
     * @param left Left finger continuous running mode servo
     * @param right Right finger continuous running mode servo
     * @param wrist Wrist servo that rotates 0 to 180 with 90 as default
     * @param gamepad Tool gamepad
     */
    public Hand(CRServo left, CRServo right, Servo wrist, Gamepad gamepad)
    {
        this.left = left;
        this.right = right;
        this.wrist = wrist;
        this.wrist.setPosition(CTR_POS);
        this.gamepad = gamepad;
        this.ignoreGamepad = false;
        this.stopThread = new Thread();
    }

    /**
     * Method that toggles if we are ignoring gamepad input (true for autonomous)
     * @param ignoreGamepad
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }

    /**
     * Method to reset the stop continuous servos thread
     * @param duration_ms how long before turning off the servos in milliseconds
     */
    private void resetStopThread(long duration_ms)
    {
        // Cancel old thread
        stopThread.interrupt();

        // Start a thread that stops the servos
        stopThread = new Thread(() -> {
            try {
                sleep(duration_ms);
                left.setPower(NO_POWER);
                right.setPower(NO_POWER);
            } catch (InterruptedException e) {
            }
        });
        stopThread.start();
    }

    /**
     * Run a continuous rotation servo for a set amount of time
     *
     * @param leftPower [0,0.5) release, [0.5] stop, (0.5,1.0] intake
     * @param rightPower [0,0.5) release, [0.5] stop, (0.5,1.0] intake
     * @param duration_ms milliseconds of wait
     */
    private void startServosForTime(double leftPower, double rightPower, long duration_ms)
    {
        // Start the servos
        if (leftPower < 0) {
            left.setDirection(CRServo.Direction.REVERSE);
        } else {
            left.setDirection(CRServo.Direction.FORWARD);
        }

        if (rightPower < 0) {
            right.setDirection(CRServo.Direction.REVERSE);
        } else {
            right.setDirection(CRServo.Direction.FORWARD);
        }

        left.setPower(Range.clip(Math.abs(leftPower), MIN_POWER, MAX_POWER));
        right.setPower(Range.clip(Math.abs(rightPower), MIN_POWER, MAX_POWER));

        // Schedule the stop (always in the future)
        resetStopThread(Math.abs(duration_ms));
    }

    /**
     * Method to set the wrist position.  0 rotates 90 degrees counter clockwise, 0.5 rotates to
     * center and 1.0 rotates 90 degrees clockwise.  This method should be called from camera
     * code to align the tool with the sample.
     *
     * @param wristPosition [0,1.0]
     */
    public void setWrist(double wristPosition)
    {
        wrist.setPosition(Range.clip(wristPosition, MIN_POS, MAX_POS));
    }

    /**
     * Run the finger servos backwards to eject the sample.  Could be called from camera code
     * once we see the bucket in the correct spot.
     *
     * @param ms milliseconds to run the servos
     */
    public void release(long ms)
    {
        // Since we are always rotated clockwise we run the right servo (lower servo)
        // a little faster to get the sample to jump up a little bit when ejected.
        startServosForTime( RELEASE_POWER,-RELEASE_POWER, ms);
    }

    /**
     * Run the finger servos forwards to intake the sample.  This method should be called
     * from camera code once we are aligned with the sample.
     *
     * @param ms milliseconds to run the servos
     */
    public void grab(long ms)
    {
        startServosForTime(-GRAB_POWER, GRAB_POWER, ms);
    }

    /**
     * Run the servos in opposite directions to rotate the sample.  Code called by the tool
     * operator to manually align the sample for hanging.  Could use the camera eventually.
     *
     * @param ms milliseconds to run the servos
     * @param left Rotate the sample to the left else rotate right
     */
    public void rotate(long ms, boolean left)
    {
        if(left)
            startServosForTime(NO_POWER - SPIN_POWER, NO_POWER + SPIN_POWER / 2, ms);
        else
            startServosForTime(NO_POWER + SPIN_POWER / 2, NO_POWER - SPIN_POWER, ms);
    }

    /**
     * Rotates the wrist such that a sample can be hung
     * */
    public void hangSample()
    {
        wrist.setPosition(CTR_POS);
    }

    /**
     * Rotates the wrist such that a sample can be ejected into a bucket
     */
    public void bucket()
    {
        wrist.setPosition(MAX_POS);
    }

    /**
     * Thread to listen for button presses on the gamepad
     */
    @Override
    public void run() {
        while (!isInterrupted()) {
            if(!ignoreGamepad) {
                // Left bumper rotates the sample counter clockwise
                if (gamepad.left_bumper) {
                    rotate(SPIN_MS, true);
                }
                // Right bumper rotates the sample clockwise
                else if (gamepad.right_bumper) {
                    rotate(SPIN_MS, false);
                }
                // Left trigger runs sample intake
                else if (gamepad.left_trigger > MIN_TRIGGER) {
                    release(RELEASE_MS);
                }
                // Right trigger runs sample release
                else if (gamepad.right_trigger > MIN_TRIGGER) {
                    grab(INTAKE_MS);
                }

                // DPAD-Up forces a sample vertical to hang
                if (gamepad.dpad_up) {
                    hangSample();
                }
                // DPAD-Down forces a sample flat for bucket
                else if (gamepad.dpad_down) {
                    bucket();
                }
                // DPAD-left manually rotate sample left
                else if (gamepad.dpad_left) {
                    wrist.setPosition(Range.clip(wrist.getPosition()-MANUAL_SPIN_INCREMENT, MIN_POS, MAX_POS));
                }
                // DPAD-right manually rotate sample right
                else if (gamepad.dpad_right) {
                    wrist.setPosition(Range.clip(wrist.getPosition()+MANUAL_SPIN_INCREMENT, MIN_POS, MAX_POS));
                }

                // If the user pressed a button for hang or bucket, get sample ready
                if(gamepad.x)
                    hangSample();
                else if(gamepad.y)
                    bucket();
            }

            // Short sleep to keep this loop from saturating
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }
}
