package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends StandardSetupOpMode {

    HuskyLens huskyLens;
    ColorCamera colorCamera;

    // center of the screen
    public static final int CENTER_X = 160;
    public static final int CENTER_Y = 180;
    public static final int CLOSE_ENOUGH = 5;

    // y=mx+b where y is inches and x is the y pixel location
    private static final double M = -11.429;
    private static final double B = 2000.0;

    private static final double INCHES_PER_PIXEL_Y = (double) 19 / 240;
    private static final double ROTATIONS_PER_INCH_Y = (double) Arm.MAX_POS/20.0;

    private HuskyLens.Block closestBlock;
    private int closestBlockX;
    private int closestBlockY;


    @Override
    public void runOpMode() throws InterruptedException {
        // get the camera from the hardware map
        colorCamera = new ColorCamera(hardwareMap, COLOR.BLUE);

        // check to see if the device is working
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // start the color recognition algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Attempt to get image data
        // TODO - plug husky into any ic2 port except 0
        byte[] data = huskyLens.getDeviceClient().read(0x30); // 0x30 is REQUEST_PHOTO, 0x31 is SEND_PHOTO, 0x38 is SEND_SCREENSHOT, 0x39 is SAVE_SCREENSHOT
        telemetry.addData("Byte from REQUEST_PHOTO", data.length);

        super.runOpMode();

        /*
        // Create a PID controller for the arm (shooting for 0 error in y)
        // TODO - Tune this First (disable turnController while tuning)
        PIDController armController = new PIDController(0.06, 0.06, 0.015);
        armController.setSetpoint(0);

        // Create a PID controller for turning (shooting for 0 error in x)
        // TODO - Tune this second (disable armController while tuning)
        PIDController turnController = new PIDController( 1.0, 0.1, 0.01);
        turnController.setSetpoint(0);

         */

        // Put the shoulder into search height position with very little holding power
        shoulder.setMode(Shoulder.Mode.SEARCH);
        //shoulder.setPosition(0.3, Shoulder.Mode.SEARCH.armInPos());

        // We could also do this by mapping the error we measure directly into
        // a correction.  That may be good enough instead of needing PIDs.
        boolean pressed = false;
        boolean xDone = false;
        boolean yDone = false;
        while(opModeIsActive()) {

            // While a is pressed on the tool gamepad
            if(gamepad2.a) {
                // the color tracking algorithm returns a block around the location of the colored object
                // from that block, we can figure out the location of our object

                    // If this is the first time we've seen the button pressed
                    // Reset the PID
                    /*
                    if(!pressed) {
                        pressed = true;
                        xDone = false;
                        yDone = false;
                        armController.reset();
                        turnController.reset();
                    }

                    if(!xDone) {
                        // Measure the current block error (x dimension)
                        int blockCenterX = block.x;
                        double errorX = blockCenterX - CENTER_X;
                        xDone = Math.abs(errorX) < CLOSE_ENOUGH;

                        // If we're not close enough
                        if(!xDone) {
                            xDone = true;
                            // Convert error to a rotation angle with the PID
                            double turnAngle = turnController.calculate(errorX);


                            // Rotate the robot to align the tool
                            // TODO - scale this by how extended the arm is
                            //motion.rotation(
                            //        (turnAngle<0) ? Motion.Direction.LEFT : Motion.Direction.RIGHT,
                            //        Math.abs(turnAngle), 0.1);
                        }
                    }

                     */

                    if(!yDone) {
                        HuskyLens.Block[] blocks = huskyLens.blocks();
                        telemetry.addData("Block count", blocks.length);
                        HuskyLens.Block block = colorCamera.getClosestBlock();
                        // we found a block
                        if (block != null) {
                            telemetry.addData("Closest Block", block.toString());
                            int blockCenterY = block.y;
                            double errorY = CENTER_Y - blockCenterY;
                            telemetry.addData("errorY", errorY);
                            //yDone = Math.abs(errorY) < CLOSE_ENOUGH;

                            // If we're not close enough
                            //if(!yDone) {
                            // Convert error to an arm extension or retraction
                            //double armPower = armController.calculate(errorY);

                            // TODO - if the arm is fully extended/retracted
                            // we actually need to motion forward/backward

                            // Drive the arm while there is still error
                            //int ticks = (int)Math.round(errorY*INCHES_PER_PIXEL_Y*ROTATIONS_PER_INCH_Y);
                            int ticks = (int) Math.round((blockCenterY * M + B));
                            telemetry.addData("blockCenterY", blockCenterY);
                            telemetry.addData("M", M);
                            telemetry.addData("Mx", blockCenterY*M);
                            telemetry.addData("B", B);
                            telemetry.addData("ticks", ticks);
                            telemetry.addData("Arm Pos", arm.getCurrentPosition());
                            telemetry.update();
                            arm.setPosition(0.3*Math.signum(ticks), arm.getCurrentPosition()+ticks);


                        }
                        yDone = true;
                        // Measure current block error (y dimension)



                            /*
                            if (armPower < 0)
                                arm.gotoMin(Math.abs(armPower));
                            else if (armPower > 0)
                                arm.gotoMax(armPower);

                             */


                        //}
                    }

                    // TODO - add logic to spin the wrist
                    // This is either openCV on image data
                    // Or switching to line detection mode
                    // And getting arrows if that's quick enough
                    // If it's too slow we'll need to go back to
                    // USB camera until we get a limelight 3a

                    // Moving one could cause error in the other, check again
                    //xDone = yDone = xDone && yDone;
                }  else {
                yDone = false;
                // if we don't find the block
            }

                // TODO - If we're all done, plunge and grab
                    // To avoid saturating the loop and let our PID's work
                    Thread.sleep(100); // Delay for next iteration

            /*
            }
            else if (pressed){
                pressed = false;
                //arm.halt();
            }

             */


        }

        waitForCompletion();
    }
}
