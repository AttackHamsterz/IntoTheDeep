package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public abstract class BodyPart extends Thread{
    // Gamepad variables
    protected Gamepad gamepad;
    protected boolean ignoreGamepad = false;

    // Motor overload protection
    protected static final long MOTOR_CHECK_PERIOD_MS = 250;  // Check 4 times a second
    protected static final int CLOSE_ENOUGH_TICKS = 10; // Turn off the other motor when we are close
    protected Thread protectionThread = new Thread();

    // Loop saturation protection
    protected static final long LOOP_PAUSE_MS = 50;

    // Consumer pattern objects
    private List<Consumer<Boolean>> listeners = new ArrayList<>();

    // Force implementing classes to implement the run class
    // This implements the things this body part can do in parallel with
    // other body parts
    @Override
    public abstract void run();

    /**
     * This method should put all motors in this body part into a safe hold mode that uses low
     * power.  It's up to each body part to do that for itself.
     * @param position Tasked position for use in the setHold method
     */
    public abstract void safeHold(int position);

    /**
     * Get the current body part position for use in safe-ing motors.
     * @return the position in ticks
     */
    public abstract int getCurrentPosition();

    /**
     * Allows a user to toggle using the gamepad
     * @param ignoreGamepad true to ignore gamepad inputs
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }

    /**
     * This sets up motor protection to avoid overloading motors.  It needs the expected position.
     * Be careful, if we can never get to the target position this will never safe!
     */
    protected void protectMotors(int targetPosition)
    {
        // Cancel old thread
        protectionThread.interrupt();

        // Start a thread that performs safe hold when the time is right
        Integer position = new Integer(targetPosition);
        protectionThread = new Thread(() -> {
            try{
                while(Math.abs(getCurrentPosition()-position) > CLOSE_ENOUGH_TICKS) {
                    sleep(MOTOR_CHECK_PERIOD_MS);
                }
                safeHold(targetPosition);
                notifyListeners();
            } catch (InterruptedException e) {
            }
        });
        protectionThread.start();
    }

    public void addListener(Consumer<Boolean> listener)
    {
        listeners.add(listener);
    }
    public void removeListener(Consumer<Boolean> listener)
    {
        listeners.remove(listener);
    }
    private void notifyListeners()
    {
        // We are done, send running false to all registered consumers
        listeners.forEach(listener -> listener.accept(false));
    }
}
