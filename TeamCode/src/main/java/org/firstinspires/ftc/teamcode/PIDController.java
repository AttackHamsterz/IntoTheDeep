package org.firstinspires.ftc.teamcode;

/**
 * Start with Ki and Kd set to zero
 * Begin by tuning the proportional gain Kp alone. This will help you
 * understand the system's response.
 * Increase Kp until you observe a reasonable response without excessive
 * overshoot. A common starting point is to set Kp to a small value,
 * such as 0.1 or 1.0, and increase it gradually.
 * Once you have a satisfactory Kp, introduce Ki. A good starting point
 * might be 0.01 or 0.1. The integral term helps eliminate steady-state
 * error, but too high a value can lead to instability and oscillations.
 * Finally, adjust Kd to dampen the system's response and reduce overshoot.
 * A starting value might be 0.01 or 0.1. Increasing Kd can help stabilize
 * the system but may slow down the response.
 *
 * Ziegler-Nichols Method:
 * This is a popular method for tuning PID controllers. It involves setting
 * Ki and Kd to zero and increasing Kp until you reach the "ultimate gain"
 * Ku where the output starts to oscillate consistently. The period of
 * oscillation Tu can then be used to calculate the PID parameters:
 *   Kp = 0.6 * Ku
 *   Ki = 2 * Kp / Tu
 *   Kd = Kp * Tu / 8
 *
 * For simple systems like motor controls
 *   Kp = 1.0, Ki = 0.1, Kd = 0.01
 */
public class PIDController {
    private final double kp; // Proportional gain
    private final double ki; // Integral gain
    private final double kd; // Derivative gain
    private double setpoint; // Desired target distance
    private double integral; // Integral term
    private double previousError; // Previous error term
    private long previousTime; // Time of the previous update

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = System.currentTimeMillis();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double currentDistance) {
        long currentTime = System.currentTimeMillis();
        double elapsedTime = (currentTime - previousTime) / 1000.0; // Convert milliseconds to seconds

        // Calculate error as distance to target
        double error = setpoint - currentDistance;

        // Proportional term
        double pTerm = kp * error;

        // Integral term
        integral += error * elapsedTime;
        double iTerm = ki * integral;

        // Derivative term
        double derivative = (error - previousError) / elapsedTime;
        double dTerm = kd * derivative;

        // Compute the total output as motor power
        double output = pTerm + iTerm + dTerm;

        // Ensure output is within motor power limits (0-100%)
        output = Math.max(-1.0, Math.min(output, 1.0));

        // Save current error and time for next calculation
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousTime = System.currentTimeMillis();
    }
}

