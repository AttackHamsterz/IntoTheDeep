package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends StandardSetupOpMode {

    Eye cameraRunner = new Eye();
    // Arm calibration values
    private static final int NEAR_Y = 186;
    private static final int FAR_Y = 66;
    private static final int NEAR_TICKS = 0;
    private static final int FAR_TICKS = 680;

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
/*
            // This block helps us calibrate (disable once calibrated)
            HuskyLens.Block block = camera.getClosestBlock();
            if (block != null) {
                telemetry.addData("Closest Block", block.toString());
                int blockCenterY = block.y;
                arm.debugTelemetry(telemetry);
                legs.debugTelemetry(telemetry);
                telemetry.update();
                legs.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                arm.halt(); // Lets us manually tug the arm for measurements
            }

 */

            if(gamepad2.a) {
                /*
                HuskyLens.Block block = camera.getClosestBlock();
                if (block != null) {
                    // Set new arm position!
                    int ticks = (int) Math.round((M * (double)block.y + B));
                    telemetry.addData("M", M);
                    telemetry.addData("blockCenterY", block.y);
                    telemetry.addData("B", B);
                    telemetry.addData("deltaTicks", ticks);
                    telemetry.addData("Arm Pos", arm.getCurrentPosition());
                    arm.setPosition(0.3, arm.getCurrentPosition() + ticks);

                 */
                cameraRunner.moveArmToColor();
                    // TODO - Set new legs position!
                /*
                    telemetry.addData("blockCenterX", block.x);
                    double ySlope = block.y * SHIFT_M + SHIFT_B;
                    double shift = (block.x-CENTER_X) * ySlope;

                    telemetry.addData("shift near m", SHIFT_NEAR_M);
                    telemetry.addData("shift far m", SHIFT_FAR_M);
                    telemetry.addData("shift m", SHIFT_M);
                    telemetry.addData("shift b", SHIFT_B);
                    telemetry.addData("y slope", ySlope);
                    telemetry.addData("shift", shift);

                    if (!pressed) {
                        legs.moveLeft(shift);
                        pressed = true;
                    } else {
                        pressed = false;
                    }

                 */

                    // TODO - Add logic to spin the wrist
                    /*
                    HuskyLens.Arrow closestArrow = camera.getClosestArrowToBlock(block);
                    telemetry.addData("Closest line", closestArrow);
                    telemetry.addData("Angle of Line", camera.findAngleOfArrow(closestArrow));

                     */
                    // This is either openCV on image data
                    // Or switching to line detection mode
                    // And getting arrows if that's quick enough
                    // If it's too slow we'll need to go back to
                    // USB camera until we get a limelight 3a
                    telemetry.update();

            }



            // Short sleep to keep this loop from saturating
            sleep(BodyPart.LOOP_PAUSE_MS);
        }
        waitForCompletion();
    }
}
