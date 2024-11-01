package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;

    @TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
    public class ColorCamera extends LinearOpMode {

        private HuskyLens huskyLens;
        // variable to store the color id we don't want to look for
        private int excludeColorId;
        // the largest height we'd expect from a valid game piece
        private int maxHeight;
        // the largest width we'd expect from a valid game piece
        private int maxWidth;
        // center of the screen
        public static final int CENTER_X = 160;
        public static final int CENTER_Y = 120;
        public static final int CLOSE_ENOUGH = 5;

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

        public ColorCamera(HuskyLens huskyLens, String colorExclude) {
            this.huskyLens = huskyLens;
            // sets the color that we are looking for
            if (colorExclude.equalsIgnoreCase("red")) {
                excludeColorId = 2;
            } else if (colorExclude.equalsIgnoreCase("blue")) {
                excludeColorId = 3;
            } else {
                throw new IllegalArgumentException("Please input a valid color");
            }
        }



        /**
         * Gets the block that is closest to the camera and has the same color as we are looking for
         * @return The closest block or null if there are no blocks
         */
        public HuskyLens.Block getClosestBlock() {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
            for (HuskyLens.Block block : blocks) {
                // add any yellow blocks
                if (block.id == 1) {
                    blocksOnScreen.add(block);
                }
            }
            if (blocksOnScreen.isEmpty()) {
                for (HuskyLens.Block block : blocks) {
                    // look for any blocks that are not the color we are excluding
                    if (block.id != excludeColorId) {
                        blocksOnScreen.add(block);
                    }
                }
            }
            // make sure there are blocks on the screen with the color we are looking for
            if (!blocksOnScreen.isEmpty()) {
                // 400 is the longest distance possible on the camera screen
                double smallestDist = 400;
                int smallestDistIndex = 0;
                // use pythagorean theorem to draw a line from the camera to each block's position
                // whichever block has the shortest distance is the block we are closest to
                for (int i = 0; i < blocksOnScreen.size(); i++) {
                    // get the distance of the block from the center of the screen
                    HuskyLens.Block block = blocksOnScreen.get(i);
                    int blockCenterX = block.x + block.width / 2;
                    int blockCenterY = block.y + block.height / 2;
                    int xDiff = Math.abs(CENTER_X - blockCenterX);
                    int yDiff = Math.abs(CENTER_Y - blockCenterY);

                    double dist = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

                    if (dist < smallestDist) {
                        smallestDist = dist;
                        smallestDistIndex = i;
                    }
                }

                return blocksOnScreen.get(smallestDistIndex);
            }
            return null;
        }

        public ArrayList<HuskyLens.Block> getAllBlocks () {
           ArrayList<HuskyLens.Block> blocks = new ArrayList<>();
            blocks.addAll(Arrays.asList(huskyLens.blocks()));
            return blocks;
        }

        @Override
        public void runOpMode() {
            // get the camera from the config
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

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

            // Create a PID controller for the arm (shooting for 0 error in y)
            // TODO - Tune this First (disable turnController while tuning)
            PIDController armController = new PIDController(1.0, 0.1, 0.01);
            armController.setSetpoint(0);

            // Create a PID controller for turning (shooting for 0 error in x)
            // TODO - Tune this second (disable armController while tuning)
            PIDController turnController = new PIDController( 1.0, 0.1, 0.01);
            turnController.setSetpoint(0);

            // Put the shoulder into search height position with very little holding power
            shoulder.setPosition(0.1, Shoulder.SEARCH_HEIGHT);

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
                    HuskyLens.Block[] blocks = huskyLens.blocks();
                    telemetry.addData("Block count", blocks.length);
                    HuskyLens.Block block = getClosestBlock();
                    if (block != null) {
                        telemetry.addData("Closest Block", block.toString());
                        // If this is the first time we've seen the button pressed
                        // Reset the PID
                        if(!pressed) {
                            pressed = true;
                            xDone = false;
                            yDone = false;
                            armController.reset();
                            turnController.reset();
                        }

                        if(!xDone) {
                            // Measure the current block error (x dimension)
                            int blockCenterX = block.x + block.width / 2;
                            double errorX = blockCenterX - CENTER_X;
                            xDone = Math.abs(errorX) < CLOSE_ENOUGH;

                            // If we're not close enough
                            if(!xDone) {
                                // Convert error to a rotation angle with the PID
                                double turnAngle = turnController.calculate(errorX);

                                // Rotate the robot to align the tool
                                // TODO - scale this by how extended the arm is
                                //motion.rotation(
                                //        (turnAngle<0) ? Motion.Direction.LEFT : Motion.Direction.RIGHT,
                                //        Math.abs(turnAngle), 0.1);
                            }
                        }

                        if(!yDone) {
                            // Measure current block error (y dimension)
                            int blockCenterY = block.y + block.height / 2;
                            double errorY = blockCenterY - CENTER_Y;
                            yDone = Math.abs(errorY) < CLOSE_ENOUGH;

                            // If we're not close enough
                            if(!yDone) {
                                // Convert error to an arm extension or retraction
                                double armPower = armController.calculate(errorY);

                                // TODO - if the arm is fully extended/retracted
                                // we actually need to motion forward/backward

                                // Drive the arm while there is still error
                                if (armPower < 0)
                                    arm.gotoMin(Math.abs(armPower));
                                else if (armPower > 0)
                                    arm.gotoMax(armPower);
                            }
                        }

                        // TODO - add logic to spin the wrist
                        // This is either openCV on image data
                        // Or switching to line detection mode
                        // And getting arrows if that's quick enough
                        // If it's too slow we'll need to go back to
                        // USB camera until we get a limelight 3a

                        // Moving one could cause error in the other, check again
                        xDone = yDone = xDone && yDone;
                    }

                    // TODO - If we're all done, plunge and grab

                    try {
                        // To avoid saturating the loop and let our PID's work
                        Thread.sleep(100); // Delay for next iteration
                    } catch (InterruptedException e) {
                    }
                }
                else if (pressed){
                    pressed = false;
                    arm.halt();
                }
                telemetry.update();
            }
        }

    }
