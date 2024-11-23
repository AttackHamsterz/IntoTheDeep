package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tail extends Thread{
    private Servo tail;
    private Thread stopThread;

    private boolean ignoreGamepad;
    private final Gamepad gamepad;

    private static final double MIN_POS = 0.0; // -90 degrees
    private static final double MAX_POS = 1.0; // +90 degrees
    private static final double CTR_POS = 0.5; // 0 degrees


    private boolean isRotated = false;

    public Tail(HardwareMap hardwareMap, Gamepad gamepad) {
        this.tail = hardwareMap.get(Servo.class, "tailServo");
        this.gamepad = gamepad;
        this.ignoreGamepad = false;
        this.stopThread = new Thread();
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
     * @param wristPosition [0,1.0]
     */
    public void setTail(double wristPosition)
    {
        tail.setPosition(Range.clip(wristPosition, MIN_POS, MAX_POS));
    }

    @Override public void run() {
        // rotate the tail when the corresponding button is pressed
        // if the tail is already rotated, rotate the tail back to its initial position
        if (gamepad.back) {
            if (!isRotated) {
                setTail(MAX_POS);
                isRotated = true;
            } else {
                setTail(CTR_POS);
                isRotated = false;
            }
        }
    }
}
