package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**-
 * This tail will wag the dog!
 */
public class Tail extends Thread{
    private final Servo tail;
    private final Shoulder shoulder;
    private final Arm arm;

    private boolean ignoreGamepad;
    private final Gamepad gamepad;

    private static final double MIN_POS = 0.0; // 0 degrees
    private static final double MAX_POS = 1.0; // +90 degrees

    public Tail(HardwareMap hardwareMap, Gamepad gamepad, Shoulder shoulder, Arm arm) {
        this.tail = hardwareMap.get(Servo.class, "tailServo"); //ch4 Servo
        this.gamepad = gamepad;
        this.ignoreGamepad = false;
        this.shoulder = shoulder;
        this.arm = arm;
    }

    /**
     * Method adds important things to telemetry
     * @param telemetry
     */
    public void debugTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Tail Position", tail.getPosition());
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
     * Method to set the tail position.  0 rotates 90 degrees counter clockwise, 0.5 rotates to
     * center and 1.0 rotates 90 degrees clockwise.  This method should be called from camera
     * code to align the tool with the sample.
     *
     * @param tailPosition [0,1.0]
     */
    public void setTail(double tailPosition)
    {
        tail.setPosition(Range.clip(tailPosition, MIN_POS, MAX_POS));
    }

    public void tailUp(){
        setTail(MIN_POS);
    }

    public void tailDown(){
        setTail(MAX_POS);
    }

    @Override public void run()
    {
        tailUp();

        if(!ignoreGamepad)
        {
            int lifting = 0;
            while (!isInterrupted())
            {
                // The following implements the lifting finite state machine.  The procedure
                // starts after listening for back and start pressed at the same time.
                // The lift runs in four parts:
                // Ready for lift (arms up)
                // The first lift (click into first bar)
                // The second lift (reach and pull to second bar)
                // Finally the bend (bend to tuck into upper pocket)
                // The user needs to press start to continue on with each state.
                // They may also press back to repeat the last action.

                // If we were lifting and start and back are pressed, reset the FSM
                if(lifting > 0 && gamepad.back && gamepad.start)
                    lifting = 0;

                // If start and back are pressed lets run the lift procedure
                if (lifting == 0) {
                    if (gamepad.back && gamepad.start) {
                        shoulder.setMode(Shoulder.Mode.NONE);
                        lifting = 1;
                    } else if (gamepad.start && !gamepad.back) {
                        setTail(MIN_POS);
                    }
                }

                // Get ready for the lift
                if(lifting == 1)
                {
                    Action shoulderUpAction = telemetryPacket -> {
                        shoulder.setPosition(1.0, Shoulder.Mode.HANG.armOutPos());
                        return false;
                    };
                    Action armUpAction = telemetryPacket -> {
                        arm.setPosition(1.0,650);
                        return false;
                    };
                    Action readyLiftAction = new SequentialAction(
                            new CompleteAction(shoulderUpAction, shoulder),
                            new CompleteAction(armUpAction, arm)
                    );
                    Actions.runBlocking(readyLiftAction);
                    lifting++;
                }

                // First pull-up
                // The robot needs to be against the rail
                // Arm extended such that fingers will grab
                // Shoulder all the way up
                if(lifting == 3) {
                    tailDown();
                    Action armDownAction = telemetryPacket -> {
                        arm.gotoMin(1.0);
                        return false;
                    };
                    Action armHangPosition = telemetryPacket -> {
                        arm.setPosition(0.6, 650);
                        return false;
                    };
                    Action firstPull = new SequentialAction(
                            new CompleteAction(armDownAction, arm),
                            new SleepAction(0.2),
                            armHangPosition
                    );
                    Actions.runBlocking(firstPull);
                    lifting++;
                }

                // Second pull-up
                // Robot needs to be hanging on static arms
                if(lifting == 5)
                {
                    Action armUpAction = telemetryPacket -> {
                        arm.setPosition(1.0, Arm.MAX_POS);
                        shoulder.setMode(Shoulder.Mode.NONE);
                        shoulder.setPosition(1.0, 2213);
                        return false;
                    };
                    Action armDownAction = telemetryPacket -> {
                        arm.gotoMin(0.8);
                        return false;
                    };
                    Action armHangPosition = telemetryPacket -> {
                        arm.setPosition(1.0, 150);
                        return false;
                    };
                    Action prePull = new SequentialAction(
                            new CompleteAction(armUpAction, arm)
                    );
                    Actions.runBlocking(prePull);
                    shoulder.setMode(Shoulder.Mode.HANG);
                    Action secondPull = new SequentialAction(
                            new SleepAction(0.5),
                            new CompleteAction(armDownAction, arm),
                            new CompleteAction(armHangPosition, arm)
                    );
                    Actions.runBlocking(secondPull);
                    lifting++;
                }

                // Final bend
                if(lifting == 7) {
                    shoulder.setMode(Shoulder.Mode.NONE);
                    tailUp();
                    Action bendAction = telemetryPacket -> {
                        shoulder.setPosition(0.5, 280);
                        return false;
                    };
                    Action finalAction = new SequentialAction(
                            new CompleteAction(bendAction, shoulder)
                    );
                    Actions.runBlocking(finalAction);
                    lifting++;
                }

                // Progress or repeat state
                if(lifting > 1 && (lifting % 2 == 0)) {
                    // Repeat last state
                    if (gamepad.back)
                        lifting--;

                    // Onto next state
                    if (gamepad.start)
                        lifting++;
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
}
