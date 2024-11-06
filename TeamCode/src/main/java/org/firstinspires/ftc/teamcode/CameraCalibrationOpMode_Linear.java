package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends LinearOpMode {

    HuskyLens huskyLens;
    ColorCamera colorCamera;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;
    private DcMotor shoulderMotor = null;
    private Servo wristServo = null;
    private CRServo leftHandServo = null;
    private CRServo rightHandServo = null;

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
// get the camera from the config
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        colorCamera = new ColorCamera(huskyLens, "red");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor
        wristServo = hardwareMap.get(Servo.class, "wristServo"); //ch0 expansion hub Servo
        leftHandServo = hardwareMap.get(CRServo.class, "leftHandServo"); //ch1 expansion hub Servo
        rightHandServo = hardwareMap.get(CRServo.class, "rightHandServo"); //ch2 expansion hub Servo

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        wristServo.setDirection(Servo.Direction.FORWARD);
        leftHandServo.setDirection(CRServo.Direction.FORWARD);
        rightHandServo.setDirection(CRServo.Direction.FORWARD);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotorLeft.setTargetPosition(0);
        armMotorRight.setTargetPosition(0);
        shoulderMotor.setTargetPosition(0);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // check to see if the device is working
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // start the color recognition algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();
        telemetry.update();

        // Threads
        Motion motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, gamepad1);
        Arm arm = new Arm(armMotorLeft, armMotorRight, gamepad2, null);
        Shoulder shoulder = new Shoulder(shoulderMotor, arm, gamepad2);
        arm.setShoulder(shoulder);
        Hand hand = new Hand(leftHandServo, rightHandServo, wristServo, gamepad2);

        motion.start();
        shoulder.start();
        arm.start();
        hand.start();

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
        motion.interrupt();
        shoulder.interrupt();
        arm.interrupt();
        hand.interrupt();
        try {
            motion.join();
            shoulder.join();
            arm.join();
            hand.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
