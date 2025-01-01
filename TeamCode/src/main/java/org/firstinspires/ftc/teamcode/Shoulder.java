package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    // 30rpm to 43rpm ratio 0.7377061597015773
    public static ArrayList<Integer> ARM_IN_POS = new ArrayList<>(Arrays.asList(
            2,  // Ground
            365,  // Search
            1073, // Low Bar
            2006, // High Bar
            1756, // Low Bucket
            2226, // High Bucket
            2490, // Hang
            105   // None is like ground
    ));

    // Shoulder positions for each mode when the arm is all the way out
    public static ArrayList<Integer> ARM_OUT_POS = new ArrayList<>(Arrays.asList(
            369,  // Ground
            502,  // Search
            879, // Low Bar
            1314, // High Bar
            1402, // Low Bucket
            2226, // High Bucket
            2490, // Hang
            369   // None is like ground
    ));

    // Number of ticks to latch a sample onto a bar
    public static int SAMPLE_HOOK_DROP = 516;

    //Variables for shoulder speed
    private static double MIN_SHOULDER_POWER = -0.9;
    private static double MAX_SHOULDER_POWER = 0.9;
    private static final double TRIM_POWER = 0.15;
    private static final double HOLD_POWER = 0.15;
    private static final double MODE_POWER = 1.0;
    private static final double DROP_POWER = -0.9;
    private static final double NO_POWER = 0.0;

    // Pre-set min and max pos based on if the arm is in or out
    // TODO - phase these out for mode values
    public static int MIN_POS_ARM_IN = Mode.GROUND.armInPos();
    public static double DELTA_MIN_POS_ARM = Mode.GROUND.armOutPos() - MIN_POS_ARM_IN;

    //Setting up vars of threading
    private final DigitalChannel shoulderSwitch;
    private final DcMotor shoulderMotor;
    private final Arm arm;

    // Var for motor counts
    private boolean hold = false;

    // Values for arm ratio and floor protection
    private boolean homed = false;
    private double armRatio;
    private static int ABSOLUTE_MIN = -250;
    private static int ABSOLUTE_MAX = Mode.HANG.armInPos();
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
    public Shoulder(HardwareMap hardwareMap, Arm arm, Gamepad gamepad, Gamepad extraGamepad) {
        // Assignments
        shoulderSwitch = hardwareMap.get(DigitalChannel.class, "shoulderSwitch"); //digital 2 control hub
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor;
        this.arm = arm;
        this.gamepad = gamepad;
        this.extraGamepad = extraGamepad;

        // Setup
        shoulderSwitch.setMode(DigitalChannel.Mode.INPUT);
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
        //telemetry.addData("Shoulder Switch", !shoulderSwitch.getState());
    }

    public Mode getMode()
    {
        return mode;
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
            setPosition(0.5, newMinPos);
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
        // Never let the shoulder go too high
        if(position > ABSOLUTE_MAX)
            position = ABSOLUTE_MAX;

        // Never let us go too low unless we've never homed
        if(homed && position < ABSOLUTE_MIN)
            position = ABSOLUTE_MIN;

        // Check the current position against the target position (do nothing if close enough)
        int currentPos = shoulderMotor.getCurrentPosition();
        if(Math.abs(currentPos - position) < CLOSE_ENOUGH_TICKS)
        {
            protectMotors(position);
            return;
        }

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

    public int getPositionForMode(Mode mode, int targetArmPosition)
    {
        return (int)Math.round(arm.getArmRatio(targetArmPosition) * (double)(mode.armOutPos() - mode.armInPos())) + mode.armInPos();
    }

    /**
     * Method lets you just set the ideal shoulder position given target arm position
     * @param mode
     * @param targetArmPosition
     */
    public void setPositionForMode(Mode mode, double power, int targetArmPosition)
    {
        setPosition(power, getPositionForMode(mode, targetArmPosition));
    }

    public boolean modeReady(Mode mode){
        return (this.mode == mode) && Math.abs(getCurrentPosition() - getPositionForMode(mode, arm.getCurrentPosition())) < (BodyPart.CLOSE_ENOUGH_TICKS * 2);
    }

    @Override
    public void run() {
        boolean pressing = false;
        while (!isInterrupted()) {

            // Reset the motor count if limit switch is triggered and current value is poor
            if(!shoulderSwitch.getState() && Math.abs(shoulderMotor.getCurrentPosition())>10){
               shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               shoulderMotor.setTargetPosition(0);
               shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               homed = true;
               continue;
            }

            // Always get the current arm ratio (use this to maintain heights given arm reach)
            armRatio = arm.getArmRatio();

            // Ground protection, sets min shoulder value based on how far the arm is out
            MIN_POS = (int) Math.round(armRatio * DELTA_MIN_POS_ARM) + MIN_POS_ARM_IN;

            // Check gamepad for user input (any input cancels the mode)
            if(!ignoreGamepad) {
                // Get current power (slower near top)
                float power = gamepad.right_stick_y;
                if(shoulderMotor.getCurrentPosition() > Mode.HIGH_BUCKET.armInPos())
                    power *= 0.5f;

                // Move the shoulder
                if (!hold && Math.abs(power) <= TRIM_POWER) {
                    shoulderMotor.setPower(0);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignore) {
                    }
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
                    int finalPosition = shoulderSwitch.getState() ? -MAX_POS : MIN_POS;
                    setPosition(power, finalPosition);
                    hold = false;
                    mode = Mode.NONE;
                }

                if(gamepad.x && !pressing) {
                    pressing = true;
                    mode = Mode.SEARCH;
                }
                else if(gamepad.a && !pressing) {
                    pressing = true;
                    mode = Mode.GROUND;
                }
                else if(gamepad.b && !pressing) {
                    pressing = true;
                    if(mode == Mode.HIGH_BAR) {
                        mode = Mode.NONE;
                        setPosition(1.0, 1143);
                    }
                    else {
                        mode = Mode.HIGH_BAR;
                        arm.setPosition(1.0, 250);
                    }
                }
                else if(gamepad.y && !pressing){
                    pressing = true;
                    if(mode == Mode.HIGH_BUCKET) {
                        arm.setPosition(1.0, 800);
                        mode = Mode.LOW_BUCKET;
                    }
                    else
                        mode = Mode.HIGH_BUCKET;
                }
                else {
                    if (!(gamepad.x || gamepad.y || gamepad.a || gamepad.b))
                        pressing = false;
                }
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
