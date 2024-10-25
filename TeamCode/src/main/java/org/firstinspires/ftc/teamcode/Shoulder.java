package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    //Variables for shoulder speed
    private static final double MIN_SHOULDER_SPEED = -1.0;
    private static final double MAX_SHOULDER_SPEED = 1.0;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.9;

    // Pre-set min and max pos based on if the arm is in or out
    public static int MIN_POS_ARM_IN = -10;
    public static int MIN_POS_ARM_OUT = -350;
    public static int MAX_POS = -1400;
    public static double DELTA_MIN_POS_ARM = (double)(MIN_POS_ARM_OUT - MIN_POS_ARM_IN);

    // Sample heights TODO - set to heights off ground and maintain given arm ratio
    public static int SAMPLE_HEIGHT_LOWER = -450;
    public static int SAMPLE_HEIGHT_UPPER = -700;
    public static int SAMPLE_HOOK_DROP = 150;

    // Bucket Heights TODO - set to heights off ground and maintain given arm ratio
    public static int LOWER_BUCKET = -844;
    public static int UPPER_BUCKET = -1242;

    // Other Heights TODO - set to height off ground and maintain given arm ratio
    public static int SEARCH_HEIGHT = -187;

    //Setting up vars of threading
    private final DcMotor shoulderMotor;
    private final Arm arm;
    private final Gamepad gamepad;

    // Var for motor counts
    private boolean ignoreGamepad = false;
    private boolean hold = false;

    // Values for arm ratio and floor protection
    private double armRatio;
    private int MIN_POS;

    /**
     * Allows a user to set or unset the gamepad buttons
     * @param ignoreGamepad
     */
    public void ignoreGamepad (boolean ignoreGamepad) {
        this.ignoreGamepad = ignoreGamepad;
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
     * This sets hold externally (forces a new hold value if false)
     * @param hold
     */
    public void setHold(boolean hold) {
        this.hold = hold;
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
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);

        // Sets the position of the shoulder
        shoulderMotor.setTargetPosition(Range.clip(position, MAX_POS, MIN_POS));
        shoulderMotor.setPower(power);
    }

    /**
     * Sets the position of the shoulder
     *
     * @param power    power of the shoulder motor
     * @param position ratio [0.0,1.0] where 0.0 is MIN_POS and 1.0 is MAX_POS
     */
    public void setPosition(double power, double position) {
        // Sets the power to the inputted power, clips the power
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);

        // Sets the position of the shoulder
        position = Range.clip(position, 0.0, 1.0);
        int pos = Range.clip( (int)Math.round(position * (double)(MAX_POS - MIN_POS))+MIN_POS, MAX_POS, MIN_POS);
        shoulderMotor.setTargetPosition(pos);
        shoulderMotor.setPower(power);
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
                    shoulderMotor.setTargetPosition(Range.clip(shoulderMotor.getCurrentPosition(), MAX_POS, MIN_POS));
                    shoulderMotor.setPower(HOLD_POWER);
                    hold = true;
                } else if (power < -TRIM_POWER) {
                    shoulderMotor.setTargetPosition(MAX_POS);
                    shoulderMotor.setPower(Math.abs(power));
                    hold = false;
                } else if (power > TRIM_POWER) {
                    shoulderMotor.setTargetPosition(MIN_POS);
                    shoulderMotor.setPower(power);
                    hold = false;
                }
            }
        }
    }
}
