package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends StandardSetupOpMode {

    // Arm calibration values
    private static final int NEAR_Y = 170;
    private static final int FAR_Y = 30;
    private static final int NEAR_TICKS = 0;
    private static final int FAR_TICKS = 1090;

    // y=mx+b where y is ticks and x is the relative y pixel location
    private static final double M = (double)(FAR_TICKS - NEAR_TICKS) / (double)(FAR_Y - NEAR_Y);
    private static final double B = (double) FAR_TICKS - (M * (double) FAR_Y);

    // Leg calibration values
    private static final int CENTER_X = 160;
    private static final int INCHES_FROM_CENTER = 4;
    private static final int SHIFT_NEAR_X = 283;
    private static final int SHIFT_NEAR_Y = 139;
    private static final double SHIFT_NEAR_M = (double)INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_NEAR_X);

    private static final int SHIFT_FAR_X = 252;
    private static final int SHIFT_FAR_Y = 109;
    private static final double SHIFT_FAR_M = (double) INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_FAR_X) ;

    // how much our slop is changing based on y
    private static final double SHIFT_M = (SHIFT_FAR_M - SHIFT_NEAR_M) / (double) (SHIFT_FAR_Y - SHIFT_NEAR_Y);
    private static final double SHIFT_B = SHIFT_NEAR_M - (SHIFT_M * (double) SHIFT_NEAR_Y);

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

        // Attempt to get image data
        //byte[] data = camera.huskyLens.getDeviceClient().read(0x30); // 0x30 is REQUEST_PHOTO - 40 bytes
        //byte[] data = camera.huskyLens.getDeviceClient().read(0x31); // 0x31 is SEND_PHOTO - 49 bytes
        //byte[] data = camera.huskyLens.getDeviceClient().read(0x38); // 0x38 is SEND_SCREENSHOT - 56 bytes
        //byte[] data = camera.huskyLens.getDeviceClient().read(0x39); // 0x39 is SAVE_SCREENSHOT - 57 bytes
        //byte[] data = camera.huskyLens.getDeviceClient().read(0x3D); // 0x39 is REQUEST_SENSOR -  bytes
        //telemetry.addData("Num Bytes", data.length);
        //telemetry.addData("Bytes:", data);

        // Put the shoulder into search height position with very little holding power
        shoulder.setMode(Shoulder.Mode.SEARCH);

        // We could also do this by mapping the error we measure directly into
        // a correction.  That may be good enough instead of needing PIDs.
        boolean pressed = false;
        while(opModeIsActive()) {

            // This block helps us calibrate (disable once calibrated)
            //HuskyLens.Block block = camera.getClosestBlock();
            //if (block != null) {
            //    telemetry.addData("Closest Block", block.toString());
            //    int blockCenterY = block.y;
            //    arm.debugTelemetry(telemetry);
            //    legs.debugTelemetry(telemetry);
            //    telemetry.update();
            //    arm.halt(); // Lets us manually tug the arm for measurements
            //}

            if(gamepad2.a && !pressed) {
                // Just update once for now
                pressed = true;

                // Grab the nearest block
                HuskyLens.Block block = camera.getClosestBlock();
                if (block != null) {
                    // Average a few arrows that fall inside our block
                    double averageAngle = 0;
                    int numAverage = 0;
                    long average_ms = 500;
                    long startTime_ms = System.currentTimeMillis();
                    camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
                    do {
                        HuskyLens.Arrow closestArrow = camera.getClosestArrowToBlock(block);
                        if(closestArrow != null)
                        {
                            // Get current angle from this arrow
                            double arrowAngle = camera.findAngleOfArrow(closestArrow);

                            // If this angle has wrapped then we put it near the average
                            // This avoids adding 89.9 + -89.9 to get an average of 0
                            // instead of either 90 or -90
                            if(numAverage>0)
                            {
                                double currentAverage = averageAngle / (double)numAverage;
                                double deltaAngle = arrowAngle - currentAverage;
                                if(deltaAngle > 160.0)
                                    arrowAngle -= 180.0;
                                else if( deltaAngle < -160.0)
                                    arrowAngle += 180.0;
                            }

                            // Increment average
                            averageAngle += arrowAngle;
                            numAverage++;
                        }
                        if(numAverage > 6)
                            break;
                    } while(System.currentTimeMillis() - startTime_ms < average_ms);
                    camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

                    // Move wrist with a good average
                    if (numAverage > 0) {
                        averageAngle /= (double)numAverage;
                        averageAngle = Range.clip(averageAngle, -90.0, 90.0);
                        double wristPos = 0.5 + (averageAngle / 180.0);
                        telemetry.addData("Average Angle", averageAngle);
                        telemetry.addData("Num in average", numAverage);
                        telemetry.addData("Wrist Pos", wristPos);
                        hand.setWrist(wristPos);
                    } else {
                        telemetry.addLine("No line average");
                    }

                    // Set new arm position!
                    int ticks = (int) Math.round((M * (double) block.y + B));
                    telemetry.addData("M", M);
                    telemetry.addData("blockCenterY", block.y);
                    telemetry.addData("B", B);
                    telemetry.addData("deltaTicks", ticks);
                    telemetry.addData("Arm Pos", arm.getCurrentPosition());
                    arm.setPosition(0.3, arm.getCurrentPosition() + ticks);

                    // Set new legs position
                    double ySlope = block.y * SHIFT_M + SHIFT_B;
                    double shift = (block.x - CENTER_X) * ySlope;
                    telemetry.addData("blockCenterX", block.x);
                    telemetry.addData("shift near m", SHIFT_NEAR_M);
                    telemetry.addData("shift far m", SHIFT_FAR_M);
                    telemetry.addData("shift m", SHIFT_M);
                    telemetry.addData("shift b", SHIFT_B);
                    telemetry.addData("y slope", ySlope);
                    telemetry.addData("shift", shift);
                    legs.moveLeft(shift);

                    // This is either openCV on image data
                    // Or switching to line detection mode
                    // And getting arrows if that's quick enough
                    // If it's too slow we'll need to go back to
                    // USB camera until we get a limelight 3a
                    telemetry.update();
                }
            }
            else
                pressed = false;



            // Short sleep to keep this loop from saturating
            sleep(BodyPart.LOOP_PAUSE_MS);
        }
        waitForCompletion();
    }
}
