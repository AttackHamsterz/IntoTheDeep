package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

public class Shoulder extends BodyPart {

    // Modes allow the shoulder to adjust to maintain constant heights from the ground.
    // Mode also provides a valid index into our tick arrays
    // Reminder - moving the shoulder with the joystick will cancel the mode
    public static enum Mode {
        GROUND(0),         // Tool is on the ground in this mode
        SEARCH(1),         // Tool is at search height in this mode
        LOW_BAR(2),        // Tool will place sample on the low bar in this mode
        HIGH_BAR(3),       // Tool will place sample on the high bar in this mode
        LOW_BUCKET(4),     // Tool will place sample in the low bucket in this mode
        HIGH_BUCKET(5),    // Tool will place sample in high bucket
        HANG(6),           // Straight Up
        NONE(7);           // Tool can do whatever it wants in this mode

        private final int value;
        private Mode(int value){
            this.value = value;
        }
        public int value(){
            return value;
        }
        public int armInPos(){
            return ARM_IN_POS.get(value);
        }
        public int armOutPos(){
            return ARM_OUT_POS.get(value);
        }
    }

    // Shoulder positions for each mode when the arm is all the way in
    public static ArrayList<Integer> ARM_IN_POS = new ArrayList<>(Arrays.asList(
            10,   // Ground
            535,  // Search
            1420, // Low Bar
            2618, // High Bar
            2894, // Low Bucket
            3059, // High Bucket
            3470, // Hang
            10    // None is like ground
    ));

    // Shoulder positions for each mode when the arm is all the way out
    public static ArrayList<Integer> ARM_OUT_POS = new ArrayList<>(Arrays.asList(
            680,  // Ground
            900,  // Search
            1300, // Low Bar
            1650, // High Bar
            1830, // Low Bucket
            3059, // High Bucket
            3470, // Hang
            680   // None is like ground
    ));

    // Number of ticks to latch a sample onto a bar
    public static int SAMPLE_HOOK_DROP = 500;

    //Variables for shoulder speed
    private static final double MIN_SHOULDER_POWER = -0.5;
    private static final double MAX_SHOULDER_POWER = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.1;
    private static final double NO_POWER = 0.0;

    // Pre-set min and max pos based on if the arm is in or out
    // TODO - phase these out for mode values
    public static int MIN_POS_ARM_IN = ARM_IN_POS.get(Mode.SEARCH.value());
    public static int MIN_POS_ARM_OUT = ARM_OUT_POS.get(Mode.SEARCH.value());
    public static int MAX_POS = ARM_IN_POS.get(Mode.HANG.value());
    public static double DELTA_MIN_POS_ARM = (double)(MIN_POS_ARM_OUT - MIN_POS_ARM_IN);

    //Setting up vars of threading
    private final DcMotor shoulderMotor;
    private final Arm arm;

    // Var for motor counts
    private boolean hold = false;

    // Values for arm ratio and floor protection
    private double armRatio;
    private int MIN_POS;

    /**
     * Constructor for the shoulder
     *
     * @param shoulderMotor the motor for the shoulder
     * @param gamepad       the gamepad used for controlling the shoulder
     */
    public Shoulder(DcMotor shoulderMotor, Arm arm, Gamepad gamepad) {
        this.shoulderMotor = shoulderMotor;
        this.arm = arm;
        this.gamepad = gamepad;
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public int getCurrentPosition() {
        return shoulderMotor.getCurrentPosition();
    }

    @Override
    public void safeHold()
    {
        // We've noticed the motors consuming lots of power while holding.  This should
        // lower the power when we don't need to move.
        shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition());
        shoulderMotor.setPower(HOLD_POWER);

        // Cancel any pending safeHolds
        protectionThread.interrupt();
    }

    /**
     * This sets hold externally (forces a new hold value if false)
     * @param hold
     */
    public void setHold(boolean hold) {
        this.hold = hold;
    }

    /**
     * This lets us tell the shoulder what our target arm ratio is so that ground protection
     * can pre-emptively move.
     * @param targetArmRatio
     */
    public void targetArmRatio(double targetArmRatio)
    {
        // Ensure arm ratio is in a valid range
        armRatio = Range.clip(targetArmRatio, 0, 1.0);

        // Convert target arm ratio to the new shoulder position
        MIN_POS = (int) Math.round(armRatio * DELTA_MIN_POS_ARM) + MIN_POS_ARM_IN;

        // Move the shoulder if necessary
        if(MIN_POS > shoulderMotor.getCurrentPosition())
            setPosition(shoulderMotor.getPower(), MIN_POS);
    }

    /**
     * Gets the ratio of shoulder position 0.0 is floor, 1.0 is straight up
     * Ratio will change based on MIN_POS set in while loop
     *
     * @return double between 0.0-1.0
     */
    public double getShoulderRatio() {
        //Sets a position variable of the robot's current position
        double pos = shoulderMotor.getCurrentPosition();
        return Range.clip( (double)(shoulderMotor.getCurrentPosition() - MIN_POS) / (double)(MAX_POS - MIN_POS), 0.0, 1.0);
    }

    /**
     * Sets the position of the shoulder
     *
     * @param power    power of the shoulder motor
     * @param position position to set the shoulder to
     */
    public void setPosition(double power, int position)
    {
        // Sets the power to the inputted power, clips the power
        power = Range.clip(power, MIN_SHOULDER_POWER, MAX_SHOULDER_POWER);
        position = Range.clip(position, MIN_POS, MAX_POS);

        // Sets the position of the shoulder
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setPower(power);

        // Generate a new motor protection thread
        protectMotors(position);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Always get the current arm ratio (use this to maintain heights given arm reach)
            armRatio = arm.getArmRatio();

            // Ground protection, sets min shoulder value based on how far the arm is out
            MIN_POS = (int) Math.round(armRatio * DELTA_MIN_POS_ARM) + MIN_POS_ARM_IN;

            // Check gamepad for user input
            if(!ignoreGamepad) {
                float power = gamepad.right_stick_y;
                if (!hold && Math.abs(power) <= TRIM_POWER) {
                    safeHold();
                    hold = true;
                } else if (power < -TRIM_POWER) {
                    // Calling setPosition here adds the motor protection, even if the driver
                    // holds the shoulder stick down forever
                    setPosition(Math.abs(power), MIN_POS);
                    hold = false;
                } else if (power > TRIM_POWER) {
                    // Calling setPosition here adds the motor protection, even if the driver
                    // holds the shoulder stick up forever
                    setPosition(power, MAX_POS);
                    hold = false;
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
