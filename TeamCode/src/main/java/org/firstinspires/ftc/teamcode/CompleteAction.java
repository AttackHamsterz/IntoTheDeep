package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Consumer;

/**
 * A CompleteAction wrapper class starts an action then waits for a body part to complete.
 */
public class CompleteAction implements Action, Consumer<Boolean> {
    private static final long DEFAULT_PROTECTION_MS = 3000;

    private final BodyPart bodyPart;
    private final Action action;
    private final long maxWait_ms;
    private boolean firstRun;
    private boolean runAction;
    private boolean waiting;

    public CompleteAction(@NonNull Action action, @NonNull BodyPart bodyPart, long maxWait_ms)
    {
        // Setup local variables
        this.action = action;
        this.bodyPart = bodyPart;
        this.maxWait_ms = maxWait_ms;
        this.firstRun = false;
        this.runAction = true;
        this.waiting = true;

        // Add myself as a consumer for the body part notifier
        bodyPart.addListener(this);
    }

    public CompleteAction(@NonNull Action action, @NonNull BodyPart bodyPart)
    {
        this(action, bodyPart, DEFAULT_PROTECTION_MS);
    }

    @Override
    public void accept(Boolean waiting) {
        // Always false (notify always means running is complete)
        this.waiting = false;
        bodyPart.removeListener(this);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // First run needs to start the wait protection timer
        if(!firstRun)
        {
            // Set a timeout thread to avoid stalling out
            Thread timeoutThread = new Thread(() -> {
                try{
                    Thread.sleep(maxWait_ms);
                } catch (InterruptedException ignored) {
                }
                if(waiting)
                    accept(false);
            });
            timeoutThread.start();
            firstRun = true;
        }

        // Some actions will run multiple times until false, those that return false are done
        if(runAction)
            runAction = action.run(telemetryPacket);

        // Once waiting is false and runAction is false we can report false
        return waiting || runAction;
    }
}
