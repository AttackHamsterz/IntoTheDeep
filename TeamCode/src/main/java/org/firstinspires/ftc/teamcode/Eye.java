package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Eye extends BodyPart{
    // Arm calibration values
    protected static final int NEAR_Y = 186;
    protected static final int FAR_Y = 66;
    protected static final int NEAR_TICKS = 0;
    protected static final int FAR_TICKS = 680;

    // y=mx+b where y is ticks and x is the relative y pixel location
    protected static final double M = (double)(FAR_TICKS - NEAR_TICKS) / (double)(FAR_Y - NEAR_Y);
    protected static final double B = (double) FAR_TICKS - (M * (double) FAR_Y);

    // Leg calibration values
    protected static final int CENTER_X = 160;
    protected static final int INCHES_FROM_CENTER = 4;
    protected static final int SHIFT_NEAR_X = 283;
    protected static final int SHIFT_NEAR_Y = 139;
    protected static final double SHIFT_NEAR_M = (double)INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_NEAR_X);

    protected static final int SHIFT_FAR_X = 252;
    protected static final int SHIFT_FAR_Y = 109;
    protected static final double SHIFT_FAR_M = (double) INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_FAR_X) ;

    // how much our slop is changing based on y
    protected static final double SHIFT_M = (SHIFT_FAR_M - SHIFT_NEAR_M) / (double) (SHIFT_FAR_Y - SHIFT_NEAR_Y);
    protected static final double SHIFT_B = SHIFT_NEAR_M - (SHIFT_M * (double) SHIFT_NEAR_Y);

    ColorCamera camera;
    Shoulder shoulder;
    Arm arm;
    Gamepad gamepad;
    StandardSetupOpMode.COLOR color;
    Telemetry telemetry;

    public Eye(HardwareMap hardwareMap, Shoulder shoulder, Arm arm, StandardSetupOpMode.COLOR color, Gamepad gamepad, Telemetry telemetry)
    {
        this.shoulder = shoulder;
        this.arm = arm;
        this.color = color;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void moveArmToColor() {
        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block block = camera.getClosestBlock();
        if (block != null) {
            // Set new arm position!
            int ticks = (int) Math.round((M * (double) block.y + B));
            telemetry.addData("M", M);
            telemetry.addData("blockCenterY", block.y);
            telemetry.addData("B", B);
            telemetry.addData("deltaTicks", ticks);
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            arm.setPosition(0.3, arm.getCurrentPosition() + ticks);
        }
    }

    public void moveLegsToColor() {

    }

    @Override
    public void run() {
        // check to see if the device is working
        if (!camera.huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + camera.huskyLens.getDeviceName());
            telemetry.update();
            return;
        }

        // start the color recognition algorithm
        //camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void safeHold(int position) {

    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }
}
