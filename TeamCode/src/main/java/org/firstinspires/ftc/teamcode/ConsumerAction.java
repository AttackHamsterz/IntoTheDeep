package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

import java.util.function.Consumer;

/**
 * A ConsumerAction wrapper class waits for a body part to complete.  Once that body part
 * is in safe hold our action is released to run.  Until then it spin waits.
 * If no action is specified then we just wait for the body part to safe and then continue.
 */
public class ConsumerAction implements Action, Consumer<Boolean> {
    private static final long STALL_PROTECTION_MS = 3000;

    private final BodyPart bodyPart;
    private final Action action;
    private boolean waiting;

    public ConsumerAction(@NonNull BodyPart bodyPart, @NonNull Action action)
    {
        // Setup local variables
        this.bodyPart = bodyPart;
        this.action = action;
        this.waiting = true;

        // Add myself as a consumer for the body part notifier
        bodyPart.addListener(this);

        // Set a timeout thread to avoid stalling out
        Thread timeoutThread = new Thread(() -> {
            try{
                Thread.sleep(STALL_PROTECTION_MS);
            } catch (InterruptedException ignored) {
            }
            if(waiting)
                accept(false);
        });
        timeoutThread.start();
    }

    public ConsumerAction(@NonNull BodyPart bodyPart)
    {
        this(bodyPart, new SleepAction(0));
    }

    @Override
    public void accept(Boolean waiting) {
        // Always false (notify always means running is complete)
        this.waiting = false;
        bodyPart.removeListener(this);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return waiting || action.run(telemetryPacket);
    }
}
