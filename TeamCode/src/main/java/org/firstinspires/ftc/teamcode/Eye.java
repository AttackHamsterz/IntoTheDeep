package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Eye extends StandardSetupOpMode {
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

    public void moveArmToColor() {
        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block block = camera.getClosestBlock();
        if (block != null) {
            // Set new arm position!
            int ticks = (int) Math.round((M * (double)block.y + B));

            double ySlope = block.y * SHIFT_M + SHIFT_B;
            double shift = (block.x-CENTER_X) * ySlope;

            legs.moveY(shift);


            // TODO - Add logic to spin the wrist
            // switch to line detection mode
            camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
            telemetry.addLine("changed algorithm");

            // get the closest arrow to our closest block
            HuskyLens.Arrow arrow = camera.getClosestArrowToBlock(block);
            // get the angle of our arrow
            double angle = camera.findAngleOfArrow(arrow);
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Call your parents!
        super.runOpMode();

        // check to see if the device is working
        if (!camera.huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + camera.huskyLens.getDeviceName());
            telemetry.update();
            return;
        }

        // start the color recognition algorithm
        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // check shoulder mode
        // if in search mode, run the code to move towards the closest sample
    }
}
