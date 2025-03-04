package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends LinearOpMode {
    protected static final double AUTO_MOVE_POWER = 1.0;

    public enum COLOR {
        RED,
        BLUE
    }
    public enum SIDE {
        LEFT,
        RIGHT
    }
    public COLOR color = COLOR.BLUE;
    public SIDE side = SIDE.RIGHT;

    // Declare OpMode members.
    protected final ElapsedTime runtime = new ElapsedTime();

    public Legs legs;
    public Arm arm;
    public Shoulder shoulder;
    public Hand hand;
    public Tail tail;
    public Eye eye;

    public boolean ignoreGamepad = false;
    public boolean favorYellow = false;

    // @Override
    public void runOpMode() throws InterruptedException {

        // Setup body parts
        /*
        CONFIGURATION
            LEGS
                ch0
                ch1
                ch2

         */
        legs = new Legs(this);
        arm = new Arm(this);
        hand = new Hand(this);
        shoulder = new Shoulder(this);
        tail = new Tail(this);
        eye = new Eye(this);
        setIgnoreGamepad(ignoreGamepad);

        // A little bit of init to not drag the tool (auto)
        if (ignoreGamepad) {
            hand.setWrist(Hand.CTR_POS);
            hand.grab(300);
            shoulder.setPosition(0.2, 111);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Launch Threads
        legs.start();
        shoulder.start();
        arm.start();
        hand.start();
        tail.start();
        eye.start();
    }

    /**
     * Method sets up this specific opmode
     * @param color color robot should use (and always yellow)
     * @param side side robot is starting on
     * @param ignoreGamepad true to ignore gamepad input
     */
    protected void setup(COLOR color, SIDE side, boolean ignoreGamepad, boolean favorYellow) {
        // Setup side and color for this opmode
        this.color = color;
        this.side = side;

        // Should we ignore the gamepad or not?
        setIgnoreGamepad(ignoreGamepad);

        // Favor yellow (for bucket work)
        this.favorYellow = favorYellow;
    }

    /**
     * Disable all gamepads for autonomous
     * @param ignoreGamepad true ignores gamepads
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
        if(legs != null) legs.setIgnoreGamepad(ignoreGamepad);
        if(shoulder != null) shoulder.setIgnoreGamepad(ignoreGamepad);
        if(arm != null) arm.setIgnoreGamepad(ignoreGamepad);
        if(hand != null) hand.setIgnoreGamepad(ignoreGamepad);
        if(tail != null) tail.setIgnoreGamepad(ignoreGamepad);
        if(eye != null) eye.setIgnoreGamepad(ignoreGamepad);
    }

    /**
     * Interrupt all the active threads and wait for them to complete
     * @throws InterruptedException failed to wait for completion
     */
    public void waitForCompletion() throws InterruptedException
    {
        legs.interrupt();
        shoulder.interrupt();
        arm.interrupt();
        hand.interrupt();
        tail.interrupt();
        eye.interrupt();

        legs.join();
        shoulder.join();
        arm.join();
        hand.join();
        tail.join();
        eye.join();
    }
}




