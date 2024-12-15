package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
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
    protected COLOR color = COLOR.BLUE;
    protected SIDE side = SIDE.RIGHT;

    // Declare OpMode members.
    protected final ElapsedTime runtime = new ElapsedTime();
    protected MecanumDrive legs;
    protected Arm arm;
    protected Shoulder shoulder;

    protected Hand hand;
    protected Tail tail;
    protected ColorCamera camera;
    protected boolean ignoreGamepad = false;
    protected boolean favorYellow = true;
    protected final Pose2d startPose = new Pose2d(0,0,0);

    // @Override
    public void runOpMode() throws InterruptedException {
        // Setup body parts
        legs = new MecanumDrive(hardwareMap, startPose, gamepad1, telemetry);
        arm = new Arm(hardwareMap, gamepad2, gamepad1, null);
        shoulder = new Shoulder(hardwareMap, arm, gamepad2, gamepad1);
        arm.setShoulder(shoulder);
        hand = new Hand(hardwareMap, gamepad2);
        tail = new Tail(hardwareMap, gamepad2, shoulder, arm);
        camera = new ColorCamera(hardwareMap, color, legs, arm, shoulder, hand, gamepad2, favorYellow);
        setIgnoreGamepad(ignoreGamepad);

        // A little bit of init to not drag the tool (auto and teleop)
        hand.grab(300);
        shoulder.setPosition(0.2, 150);

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
        camera.start();
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
        if (tail != null) tail.setIgnoreGamepad(ignoreGamepad);
        if (camera != null) camera.setIgnoreGamepad(ignoreGamepad);
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
        camera.interrupt();

        legs.join();
        shoulder.join();
        arm.join();
        hand.join();
        tail.join();
        camera.join();
    }
}




