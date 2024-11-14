package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends StandardSetupOpMode {

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
    private static final int NEAR_DX = 80;
    private static final int FAR_DX = 100;
    private static final int NEAR_TY = 3;
    private static final int FAR_TY = 6;

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
                double errorY = CENTER_Y - blockCenterY;
                telemetry.addData("errorY", errorY);
                arm.debugTelemetry(telemetry);
                legs.debugTelemetry(telemetry);
                telemetry.update();
                legs.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                arm.halt(); // Lets us manually tug the arm for measurements
            }
            */

            if(gamepad2.a) {
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

                    // TODO - Set new legs position!
                    telemetry.addData("blockCenterX", block.x);

                    // TODO - Add logic to spin the wrist
                    // This is either openCV on image data
                    // Or switching to line detection mode
                    // And getting arrows if that's quick enough
                    // If it's too slow we'll need to go back to
                    // USB camera until we get a limelight 3a
                    telemetry.update();
                }
            }

            // Short sleep to keep this loop from saturating
            sleep(BodyPart.LOOP_PAUSE_MS);
        }
        waitForCompletion();
    }
}
