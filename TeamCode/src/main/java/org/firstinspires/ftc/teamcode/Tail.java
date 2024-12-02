package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
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
        this.tail = hardwareMap.get(Servo.class, "tailServo");
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

    @Override public void run()
    {
        if(!ignoreGamepad)
        {
            int lifting = 0;
            while (!isInterrupted())
            {
                // The following implements the lifting finite state machine.  The procedure
                // starts after listening for back and start pressed at the same time.
                // The lift runs in four parts:
                // Ready for lift
                // The first lift
                // The second lift
                // Finally the bend.
                // The user needs to press start to continue on with each state.
                // They may also press back to repeat the last action.

                // If start and back are pressed lets run the lift procedure
                if(lifting == 0 && gamepad.start && gamepad.back)
                    lifting = 1;

                // Get ready for the lift
                if(lifting == 1)
                {
                    shoulder.setPosition(1.0, Shoulder.Mode.HANG.armOutPos());
                    arm.setPosition(1.0, 1000);
                    lifting++;
                }

                // First pull-up
                // The robot needs to be against the rail
                // Arm extended such that fingers will grab
                // Shoulder all the way up
                if(lifting == 3) {
                    setTail(MIN_POS);
                    arm.gotoMin(1.0);
                    lifting++;
                }

                // Second pull-up
                // Robot needs to be hanging on static arms
                if(lifting == 5)
                {
                    Action armUpAction = telemetryPacket -> {
                        arm.setPosition(1.0, 2100);
                        return false;
                    };
                    Action armDownAction = telemetryPacket -> {
                        arm.setPosition(1.0, 0);
                        return false;
                    };
                    Action armHangPosition = telemetryPacket -> {
                        arm.setPosition(1.0, 100);
                        return false;
                    };
                    Action secondPull = new SequentialAction(
                            new CompleteAction(armUpAction, arm),
                            new CompleteAction(armDownAction, arm),
                            new CompleteAction(armHangPosition, arm)
                    );
                    Actions.runBlocking(secondPull);
                    lifting++;
                }

                // Final bend
                if(lifting == 7) {
                    setTail(MIN_POS);
                    shoulder.setPosition(0.5, 400);
                    lifting++;
                }

                // State failure, repeat last one
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
