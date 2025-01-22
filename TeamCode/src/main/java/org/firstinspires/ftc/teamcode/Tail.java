package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**-
 * This tail will wag the dog!
 */
public class Tail extends Thread{
    private final Servo tail;

    private boolean ignoreGamepad;
    private final Gamepad gamepad;
    private final Gamepad otherGamepad;
    private final StandardSetupOpMode ssom;

    private static final double MIN_POS = 0.0; // 0 degrees
    private static final double MAX_POS = 1.0; // +90 degrees

    public Tail(StandardSetupOpMode ssom){
        this.ssom = ssom;
        this.tail = ssom.hardwareMap.get(Servo.class, "tailServo"); //ch4 Servo
        this.gamepad = ssom.gamepad2;
        this.otherGamepad = ssom.gamepad1;
        this.ignoreGamepad = false;
    }

    /**
     * Method adds important things to telemetry
     * @param telemetry debug
     */
    public void debugTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Tail Position", tail.getPosition());
    }

    /**
     * Method that toggles if we are ignoring gamepad input (true for autonomous)
     * @param ignoreGamepad don't allow button mashing
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
        // Always start with the tail up
        tailUp();

        // Actions we use for lifting
        Action shoulderUpAction = telemetryPacket -> {
            ssom.shoulder.setPosition(1.0, Shoulder.Mode.HANG.armOutPos());
            return false;
        };
        Action armUpAction = telemetryPacket -> {
            ssom.arm.setPosition(1.0,550);
            return false;
        };
        Action armDownAction = telemetryPacket -> {
            ssom.arm.setPosition(1.0, -20);
            return false;
        };
        Action upAction = telemetryPacket -> {
            ssom.arm.setPosition(1.0, 2200);
            ssom.shoulder.setMode(Shoulder.Mode.NONE);
            ssom.shoulder.setPosition(1.0, 2213);
            return false;
        };
        Action armHangPosition = telemetryPacket -> {
            ssom.arm.setPosition(1.0, 150);
            return false;
        };
        Action shoulderHangPosition = telemetryPacket -> {
            ssom.shoulder.setPosition(1.0, Shoulder.Mode.HANG.armOutPos());
            return false;
        };
        Action bendAction = telemetryPacket -> {
            ssom.shoulder.setPosition(0.8, 50);
            return false;
        };

        if(!ignoreGamepad)
        {
            int lifting = 0;
            while (!isInterrupted())
            {
                // If driver hits start, force tail to MIN_POS and reset lifting FSM
                if (otherGamepad.start) {
                    setTail(MIN_POS);
                    lifting = 0;
                }

                // The following implements the lifting finite state machine.  The procedure
                // starts after listening for back and start pressed at the same time.
                // The lift runs in four parts:
                // Ready for lift (arms up)
                // The first lift (click into first bar)
                // The second lift (reach and pull to second bar)
                // Finally the bend (bend to tuck into upper pocket)
                // The user needs to press start to continue on with each state.
                // They may also press back to repeat the last action.

                // If start and back are pressed lets run the lift procedure
                if (lifting == 0) {
                    if (gamepad.back && gamepad.start) {
                        ssom.shoulder.setMode(Shoulder.Mode.NONE);
                        lifting = 1;
                    }
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

                // If FSM is done, reset
                if(lifting >= 8)
                    lifting = 0;

                // Get ready for the lift
                if(lifting == 1)
                {
                    Action readyLiftAction = new ParallelAction(
                            new CompleteAction(shoulderUpAction, ssom.shoulder),
                            new CompleteAction(armUpAction, ssom.arm)
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
                    Action firstPull = new SequentialAction(
                            new CompleteAction(armDownAction, ssom.arm),
                            armUpAction
                    );
                    Actions.runBlocking(firstPull);
                    lifting++;
                }

                // Second pull-up
                // Robot needs to be hanging on static arms
                if(lifting == 5)
                {
                    Actions.runBlocking(new CompleteAction(upAction, ssom.arm));
                    //ssom.shoulder.setMode(Shoulder.Mode.HANG);
                    Action secondPull = new SequentialAction(
                            //new SleepAction(0.5),
                            new CompleteAction(shoulderHangPosition, ssom.shoulder),
                            new CompleteAction(armDownAction, ssom.arm),
                            new CompleteAction(armHangPosition, ssom.arm)
                    );
                    Actions.runBlocking(secondPull);
                    lifting++;
                }

                // Final bend
                if(lifting == 7) {
                    ssom.shoulder.setMode(Shoulder.Mode.NONE);
                    tailUp();
                    Actions.runBlocking(new CompleteAction(bendAction, ssom.shoulder));
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
