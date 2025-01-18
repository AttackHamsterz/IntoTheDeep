package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.List;


@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends StandardSetupOpMode {

    // Arm calibration values (ensure shoulder is in
    private static final int NEAR_Y = 160;
    private static final int FAR_Y = 50;
    private static final int NEAR_TICKS = 0;
    private static final int FAR_TICKS = 550;

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
        boolean running = false;
        while(opModeIsActive()) {

            // This block helps us calibrate (disable once calibrated)
            HuskyLens.Block block = camera.getClosestBlock();
            if (block != null) {
                telemetry.addData("Closest Block", block.toString());
                int blockCenterY = block.y;
                arm.debugTelemetry(telemetry);
                legs.debugTelemetry(telemetry);
                telemetry.update();
                arm.halt(); // Lets us manually tug the arm for measurements
            }

            if(gamepad2.x && shoulder.getMode() == Shoulder.Mode.SEARCH && !running) {
                // Grab the nearest block
                final HuskyLens.Block firstBlock = camera.getClosestBlock();
                if (firstBlock != null) {
                    running = true;
                    telemetry.addData("block1 id", firstBlock.id);
                    // Move the arm and robot to get that block closer
                    Action moveArmToBlock = telemetryPacket -> {
                        shoulder.setMode(Shoulder.Mode.SEARCH);
                        int ticks = (int) Math.round((M * (double) firstBlock.y + B));
                        arm.setPosition(1.0, arm.getCurrentPosition() + ticks);
                        telemetry.addData("block1 ticks", ticks);
                        return false;
                    };
                    Action strafeToBlock = telemetryPacket -> {
                        double ySlope = firstBlock.y * SHIFT_M + SHIFT_B;
                        double shift = (firstBlock.x - CENTER_X) * ySlope;
                        telemetry.addData("block1 shift", shift);
                        legs.moveLeft(shift, false);
                        return false;
                    };
                    ParallelAction centerBlockAction = new ParallelAction(
                            new CompleteAction(moveArmToBlock, arm),
                            new CompleteAction(strafeToBlock, legs));
                    Actions.runBlocking(centerBlockAction);

                    // Get the block again now that it's closer
                    final HuskyLens.Block secondBlock = camera.getClosestBlock();
                    if(secondBlock != null) {
                        double averageAngle = 0;

                        // Get a rough angle from the block itself
                        //averageAngle = Math.toDegrees(Math.atan2(secondBlock.height, secondBlock.width));
                        //telemetry.addData("Block Angle", averageAngle);

                        // Average a few arrows that fall inside our block
                        int numAverage = 0;
                        long average_ms = 500;
                        long startTime_ms = System.currentTimeMillis();
                        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
                        do {
                            List<HuskyLens.Arrow> arrows = camera.getArrowsInBlock(secondBlock);
                            for(HuskyLens.Arrow arrow : arrows){
                                // Get current angle from this arrow
                                double arrowAngle = camera.findAngleOfArrow(arrow);

                                // If this angle has wrapped then we put it near the average
                                // This avoids adding 89.9 + -89.9 to get an average of 0
                                // instead of either 90 or -90
                                if (numAverage > 0) {
                                    double currentAverage = averageAngle / (double) numAverage;
                                    double deltaAngle = arrowAngle - currentAverage;
                                    if (deltaAngle > 160.0)
                                        arrowAngle -= 180.0;
                                    else if (deltaAngle < -160.0)
                                        arrowAngle += 180.0;
                                }

                                // Increment average
                                averageAngle += arrowAngle;
                                numAverage++;
                            }
                            if (numAverage > 6)
                                break;
                        } while (System.currentTimeMillis() - startTime_ms < average_ms);
                        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


                        // Set new arm position!
                        int ticks = (int) Math.round((M * (double) secondBlock.y + B));
                        telemetry.addData("block2 ticks", ticks);
                        arm.setPosition(1.0, arm.getCurrentPosition() + ticks);

                        // Set new legs position
                        double ySlope = secondBlock.y * SHIFT_M + SHIFT_B;
                        double shift = (secondBlock.x - CENTER_X) * ySlope;
                        telemetry.addData("block2 shift", ticks);
                        legs.moveLeft(shift, false);

                        // Move wrist with a good average
                        if (numAverage > 0) {
                            averageAngle /= (double) numAverage;
                            averageAngle = Range.clip(averageAngle, -90.0, 90.0);
                            double wristPos = 0.5 - (0.5*averageAngle / 90.0);
                            telemetry.addData("Average Angle", averageAngle);
                            telemetry.addData("Num in average", numAverage);
                            telemetry.addData("Wrist Pos", wristPos);
                            hand.setWrist(wristPos);

                            // Plunge to pickup
                            Action grab = telemetryPacket -> {
                                hand.grab(1000);
                                return false;
                            };
                            Action dropShoulder = telemetryPacket -> {
                                shoulder.setMode(Shoulder.Mode.GROUND);
                                return false;
                            };
                            Action raiseShoulder = telemetryPacket -> {
                                shoulder.setMode(Shoulder.Mode.SEARCH);
                                hand.hangSample();
                                return false;
                            };
                            Action snag = new SequentialAction(
                                    dropShoulder,
                                    new CompleteAction(grab, hand),
                                    raiseShoulder);
                            Actions.runBlocking(snag);

                        } else {
                            telemetry.addLine("No line average");
                        }
                    }
                    else
                        telemetry.addLine("block2 null");

                    // This is either openCV on image data
                    // Or switching to line detection mode
                    // And getting arrows if that's quick enough
                    // If it's too slow we'll need to go back to
                    // USB camera until we get a limelight 3a
                    running = false;
                }
                else
                    telemetry.addLine("block1 null");
                telemetry.update();
            }

            // Short sleep to keep this loop from saturating
            sleep(BodyPart.LOOP_PAUSE_MS);
        }
        waitForCompletion();
    }
}
