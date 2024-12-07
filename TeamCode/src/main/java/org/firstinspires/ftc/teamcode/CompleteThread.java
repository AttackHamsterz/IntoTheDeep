package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Consumer;

/**
 * A CompleteAction wrapper class starts an action then waits for a body part to complete.
 */
public class CompleteThread implements Action {
    private static final long DEFAULT_PROTECTION_MS = 3000;

    private final Thread thread;
    private final long maxWait_ms;
    private Thread timeoutThread;
    private boolean firstRun;

    private class TimeoutThread extends Thread{
        @Override
        public void run(){
            try{
                Thread.sleep(maxWait_ms);
            } catch (InterruptedException ignored) {
            }
            thread.interrupt();
        }
    }

    public CompleteThread(@NonNull Thread thread, long maxWait_ms)
    {
        // Setup local variables
        this.thread = thread;
        this.maxWait_ms = maxWait_ms;
        this.timeoutThread = new Thread();
        this.firstRun = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(firstRun){
            firstRun = false;
            thread.start();
        }

        // Keep trying our action until the thread completes
        return thread.isAlive();
    }
}
