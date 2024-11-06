package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends LinearOpMode {
    // Declare OpMode members.
    protected final ElapsedTime runtime = new ElapsedTime();
    protected MecanumDrive legs;
    protected Arm arm;
    protected Shoulder shoulder;
    protected Hand hand;
    // TODO - add camera here
    protected boolean ignoreGamepad = false;

    // @Override
    public void runOpMode() throws InterruptedException {
        // Setup body parts
        legs = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), gamepad1);
        arm = new Arm(hardwareMap, gamepad2, null);
        shoulder = new Shoulder(hardwareMap, arm, gamepad2);
        arm.setShoulder(shoulder);
        hand = new Hand(hardwareMap, gamepad2);
        setIgnoreGamepad(ignoreGamepad);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Launch Threads
        legs.start();
        shoulder.start();
        arm.start();
        hand.start();
    }

    /**
     * Disable all gamepads for autonomous
     * @param ignoreGamepad true ignores gamepads
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        if(legs != null) legs.setIgnoreGamepad(ignoreGamepad);
        if(shoulder != null) shoulder.setIgnoreGamepad(ignoreGamepad);
        if(arm != null) arm.setIgnoreGamepad(ignoreGamepad);
        if(hand != null) hand.setIgnoreGamepad(ignoreGamepad);
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
        legs.join();
        shoulder.join();
        arm.join();
        hand.join();
    }
}




