package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class Shoulder extends BodyPart {

    // Modes allow the shoulder to adjust to maintain constant heights from the ground.
    // Mode also provides a valid index into our tick arrays
    // Reminder - moving the shoulder with the joystick will cancel the mode
    public enum Mode {
        GROUND(0),         // Tool is on the ground in this mode
        SEARCH(1),         // Tool is at search height in this mode
        LOW_BAR(2),        // Tool will place sample on the low bar in this mode
        HIGH_BAR(3),       // Tool will place sample on the high bar in this mode
        LOW_BUCKET(4),     // Tool will place sample in the low bucket in this mode
        HIGH_BUCKET(5),    // Tool will place sample in high bucket
        HANG(6),           // Straight Up
        NONE(7);           // Tool can do whatever it wants in this mode

        private final int value;

        Mode(int value){
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
            143,  // Ground
            543,  // Search
            1265, // Low Bar
            2686, // High Bar
            3225, // Low Bucket
            2921, // High Bucket
            3375, // Hang
            143   // None is like ground
    ));

    // Shoulder positions for each mode when the arm is all the way out
    public static ArrayList<Integer> ARM_OUT_POS = new ArrayList<>(Arrays.asList(
            500,  // Ground
            685,  // Search
            1047, // Low Bar
            1636, // High Bar
            1731, // Low Bucket
            2921, // High Bucket
            3375, // Hang
            500   // None is like ground
    ));

    // Number of ticks to latch a sample onto a bar
    public static int SAMPLE_HOOK_DROP = 700;

    //Variables for shoulder speed
    private static final double MIN_SHOULDER_POWER = -0.9;
    private static final double MAX_SHOULDER_POWER = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.05;
    private static final double MODE_POWER = 1.0;
    private static final double DROP_POWER = -0.9;
    private static final double NO_POWER = 0.0;

    // Pre-set min and max pos based on if the arm is in or out
    // TODO - phase these out for mode values
    public static int MIN_POS_ARM_IN = Mode.GROUND.armInPos();
    public static double DELTA_MIN_POS_ARM = Mode.GROUND.armOutPos() - MIN_POS_ARM_IN;

    //Setting up vars of threading
    private final DcMotor shoulderMotor;
    private final Arm arm;

    // Var for motor counts
    private boolean hold = false;

    // Values for arm ratio and floor protection
    private double armRatio;
    private int MIN_POS = Mode.NONE.armInPos();
    public static int MAX_POS = Mode.HANG.armInPos();

    // Mode for the shoulder
    private Mode mode = Mode.NONE;

    /**
     * Constructor for the shoulder
     *
     * @param hardwareMap map with shoulder parts
     * @param gamepad       the gamepad used for controlling the shoulder
     */
    public Shoulder(HardwareMap hardwareMap, Arm arm, Gamepad gamepad) {
        // Assignments
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor;
        this.arm = arm;
        this.gamepad = gamepad;

        // Setup
        shoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setTargetPosition(0);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.mode = Mode.NONE;
    }

    /**
     * Method adds important things to telemetry
     * @param telemetry place to put telemetry
     */
    public void debugTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Shoulder Position", "(%7d)", shoulderMotor.getCurrentPosition());
        telemetry.addData("Shoulder Power", shoulderMotor.getPower());
        telemetry.addData("Shoulder Ratio", getShoulderRatio());
    }

    /**
     * Method to drop the shoulder to hook the sample onto the bar
     */
    public void dropSample()
    {
        setMode(Mode.NONE);
        setPosition(DROP_POWER, getCurrentPosition() - SAMPLE_HOOK_DROP);
    }

    @Override
    public int getCurrentPosition() {
        return shoulderMotor.getCurrentPosition();
    }

    @Override
    public void safeHold(int position)
    {
        // We've noticed the motors consuming lots of power while holding.  This should
        // lower the power when we don't need to move.
        shoulderMotor.setPower(HOLD_POWER);
        shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition());

        // Cancel any pending safeHolds
        protectionThread.interrupt();
    }

    /**
     * This sets hold externally (forces a new hold value if false)
     * @param hold true externally forces us to hold
     */
    public void setHold(boolean hold) {
        this.hold = hold;
    }

    /**
     * This lets us tell the shoulder what our target arm ratio is so that ground protection
     * can pre-emptively move.
     * @param targetArmRatio [0,1]
     */
    public void targetArmRatio(double targetArmRatio)
    {
        // Ignore if we have listeners waiting for results
        if(getNumListeners()>0)
            return;

        // Ensure arm ratio is in a valid range
        armRatio = Range.clip(targetArmRatio, 0, 1.0);

        // Convert target arm ratio to the new shoulder position
        int newMinPos = (int) Math.round(armRatio * DELTA_MIN_POS_ARM) + MIN_POS_ARM_IN;

        // Move the shoulder if necessary
        if(newMinPos > shoulderMotor.getCurrentPosition())
            setPosition(shoulderMotor.getPower(), newMinPos);
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
        return Range.clip( (pos - (double)MIN_POS) / (double)(MAX_POS - MIN_POS), 0.0, 1.0);
    }

    /**
     * Sets the position of the shoulder
     *
     * @param power    power of the shoulder motor
     * @param position position to set the shoulder to
     */
    public void setPosition(double power, int position)
    {
        // Check the current position against the target position (do nothing if close enough)
        position = Range.clip(position, MIN_POS, MAX_POS);
        int currentPos = shoulderMotor.getCurrentPosition();
        if(Math.abs(currentPos - position) < CLOSE_ENOUGH_TICKS)
            return;

        // Ensure inputs are valid (flip sign of power for lowering)
        power = Range.clip(Math.abs(power) * Math.signum(position-currentPos), MIN_SHOULDER_POWER, MAX_SHOULDER_POWER);

        // Sets the position of the shoulder
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setPower(power);

        // Generate a new motor protection thread
        protectMotors(position);
    }

    public void setMode(Mode mode)
    {
        this.mode = mode;
    }

    /**
     * Method lets you just set the ideal shoulder position given target arm position
     * @param mode
     * @param targetArmPosition
     */
    public void setPositionForMode(Mode mode, double power, int targetArmPosition)
    {
        int newPos = (int)Math.round(arm.getArmRatio(targetArmPosition) * (double)(mode.armOutPos() - mode.armInPos())) + mode.armInPos();
        setPosition(power, newPos);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Always get the current arm ratio (use this to maintain heights given arm reach)
            armRatio = arm.getArmRatio();

            // Ground protection, sets min shoulder value based on how far the arm is out
            MIN_POS = (int) Math.round(armRatio * DELTA_MIN_POS_ARM) + MIN_POS_ARM_IN;

            // Check gamepad for user input (any input cancels the mode)
            if(!ignoreGamepad) {
                float power = gamepad.right_stick_y;
                if (!hold && Math.abs(power) <= TRIM_POWER) {
                    safeHold(shoulderMotor.getCurrentPosition());
                    hold = true;
                } else if (power < -TRIM_POWER) {
                    // Calling setPosition here adds the motor protection, even if the driver
                    // holds the shoulder stick down forever
                    setPosition(Math.abs(power), MAX_POS);
                    hold = false;
                    mode = Mode.NONE;
                } else if (power > TRIM_POWER) {
                    // Calling setPosition here adds the motor protection, even if the driver
                    // holds the shoulder stick up forever
                    setPosition(power, MIN_POS);
                    hold = false;
                    mode = Mode.NONE;
                }
                if(gamepad.a)
                    mode = Mode.SEARCH;
                else if(gamepad.b)
                    mode = Mode.GROUND;
                else if(gamepad.x)
                    mode = Mode.HIGH_BAR;
                else if(gamepad.y)
                    mode = Mode.LOW_BUCKET;
            }

            // Always satisfy the mode if no buttons were pressed
            if(mode != Mode.NONE)
            {
                int newPos = (int)Math.round(armRatio * (double)(mode.armOutPos() - mode.armInPos())) + mode.armInPos();
                setPosition(MODE_POWER, newPos);
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
