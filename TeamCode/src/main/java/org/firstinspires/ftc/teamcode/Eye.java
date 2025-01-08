package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Eye extends BodyPart {

    private static final double OFF_POWER = 0.0;
    private static final double RED_POWER = 0.279;
    private static final double YELLOW_POWER = 0.388;
    private static final double BLUE_POWER = 0.611;

    // variable to store the color of alliance
    private static final int NONE_ID = 0;
    private static final int YELLOW_ID = 1;
    private static final int RED_ID = 2;
    private static final int BLUE_ID = 3;
    private final int colorId;

    protected static final int WEBCAM_WIDTH = 640; //800;//1920;//640;//1920;
    protected static final int WEBCAM_HEIGHT = 480; //600;//1080;//480;//1080;
    public static final int TARGET_X = WEBCAM_WIDTH / 2;
    public static final int TARGET_Y = 100;

    // Arm calibration values
    protected static final int NEAR_Y = 370;
    protected static final int FAR_Y = 211;
    protected static final int NEAR_TICKS = 0;
    protected static final int FAR_TICKS = 590;

    // y=mx+b where y is ticks and x is the relative y pixel location
    protected static final double M = (double) (FAR_TICKS - NEAR_TICKS) / (double) (FAR_Y - NEAR_Y);
    protected static final double B = (double) FAR_TICKS - (M * (double) FAR_Y);

    // Leg calibration values
    protected static final int CENTER_X = 343;
    protected static final int INCHES_FROM_CENTER = 4;
    protected static final int SHIFT_NEAR_X = 501;
    protected static final int SHIFT_NEAR_Y = 294;
    protected static final double SHIFT_NEAR_M = (double) INCHES_FROM_CENTER / (double) (CENTER_X - SHIFT_NEAR_X);

    protected static final int SHIFT_FAR_X = 471;
    protected static final int SHIFT_FAR_Y = 197;
    protected static final double SHIFT_FAR_M = (double) INCHES_FROM_CENTER / (double) (CENTER_X - SHIFT_FAR_X);

    // how much our slop is changing based on y
    protected static final double SHIFT_M = (SHIFT_FAR_M - SHIFT_NEAR_M) / (double) (SHIFT_FAR_Y - SHIFT_NEAR_Y);
    protected static final double SHIFT_B = SHIFT_NEAR_M - (SHIFT_M * (double) SHIFT_NEAR_Y);

    ColorCamera camera;
    OpenCvWebcam webcam;
    MecanumDrive legs;
    Shoulder shoulder;
    Arm arm;
    Hand hand;
    Gamepad gamepad;
    StandardSetupOpMode.COLOR color;
    boolean favorYellow;
    Telemetry telemetry;
    public Servo lights;
    private FrameProcessing fp;
    private double smallestDist = 10000;
    private int smallestDistIndex = 0;
    private double inchesLeft;
    private double inchesRight;


    public Eye(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color, boolean favorYellow, MecanumDrive legs, Arm arm, Shoulder shoulder, Hand hand, Gamepad gamepad, Telemetry telemetry) {
        this.legs = legs;
        this.arm = arm;
        this.shoulder = shoulder;
        this.hand = hand;
        this.color = color;
        this.favorYellow = favorYellow;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        fp = new FrameProcessing(WEBCAM_WIDTH, WEBCAM_HEIGHT);

        this.lights = hardwareMap.get(Servo.class, "lights"); // Expansion hub ch3
        // LED setup
        colorId = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;

        if(colorId == BLUE_ID)
            lights.setPosition(BLUE_POWER);
        else
            lights.setPosition(RED_POWER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 60);
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                if(telemetry != null) {
                    telemetry.addLine("Camera failed to init");
                    telemetry.update();
                }
            }
        });
    }

    public void debugTelemetry(Telemetry telemetry)
    {
        if (fp.centerXVal.size() > 0) {
            telemetry.addData("Total num", fp.centerXVal.size());
            telemetry.addData("Winner x", fp.centerXVal.get(smallestDistIndex));
            telemetry.addData("Winner y", fp.centerYVal.get(smallestDistIndex));
            telemetry.addData("Winner a", Math.toDegrees(fp.angleVal.get(smallestDistIndex)));
            telemetry.addData("Wrist pos", fp.angleVal.get(smallestDistIndex) / Math.PI);
            telemetry.addData("Arm pos", arm.getCurrentPosition());
            if (smallestDistIndex >= 0 && smallestDist < fp.centerYVal.size()) {
                telemetry.addData("blockCenterY", fp.centerYVal.get(smallestDistIndex));
                telemetry.addData("deltaTicks", (int) Math.round((M * fp.centerYVal.get(smallestDistIndex) + B)));
                telemetry.addData("Arm Pos", arm.getCurrentPosition());
            }
        }
        if(fp.bar_left_y > 0 && fp.bar_right_y > 0) {
            telemetry.addData("Inches left", inchesLeft);
            telemetry.addData("Inches right", inchesRight);

        }
    }

    public void moveToColor() {
            // Set new arm position!
        Action moveArm = telemetryPacket -> {
            int ticks = (int) Math.round((M * fp.centerYVal.get(smallestDistIndex) + B));
            arm.setPosition(1.0, arm.getCurrentPosition() + ticks);
            //telemetry.addData("block1 ticks", ticks);
            return false;
        };
        Action strafeToBlock = telemetryPacket -> {
            double ySlope = fp.centerYVal.get(smallestDistIndex) * SHIFT_M + SHIFT_B;
            double shift = (fp.centerXVal.get(smallestDistIndex) - CENTER_X) * ySlope;
            legs.moveLeft(shift, false);
            return false;
        };
        Action grab = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.GROUND);
            hand.grab(1000);
            return false;
        };
        Action raiseShoulder = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(1.0, shoulder.getPositionForMode(Shoulder.Mode.SEARCH , arm.getCurrentPosition()) * 3 / 4);
            //hand.hangSample();
            return false;
        };


        //hand.setWrist(wristPos);

        ParallelAction premove = new ParallelAction(
                new CompleteAction(moveArm, arm),
                new CompleteAction(strafeToBlock, legs)
        );
        SequentialAction moveArmToBlock = new SequentialAction(
                premove,
                new CompleteAction(grab, hand),
                new CompleteAction(raiseShoulder, shoulder));
        Actions.runBlocking(moveArmToBlock);
        shoulder.setMode(Shoulder.Mode.NONE);
    }

    public void moveLegsToColor() {
        Action strafeToBlock = telemetryPacket -> {
            double ySlope = fp.centerYVal.get(smallestDistIndex) * SHIFT_M + SHIFT_B;
            double shift = (fp.centerXVal.get(smallestDistIndex) - CENTER_X) * ySlope;
            legs.moveLeft(shift, false);
            return false;
        };
        SequentialAction centerBlockAction = new SequentialAction(
                new CompleteAction(strafeToBlock, legs));
        Actions.runBlocking(centerBlockAction);
    }

    public void moveLegsToBar(double leftInches, double rightInches){
        shoulder.setMode(Shoulder.Mode.NONE);
        Action moveToBar = telemetryPacket -> {
            double averageInches = (leftInches + rightInches) / 2.0;
            legs.moveForward(averageInches, false);
            return false;
        };
        Action lowerShoulder = telemetryPacket -> {
            shoulder.setPosition(1.0, 1143);
            return false;
        };
        SequentialAction gotoBar = new SequentialAction(
                new CompleteAction(moveToBar, legs));
                new CompleteAction(lowerShoulder, shoulder);
        Actions.runBlocking(gotoBar);
    }

    public void plunge() {
        Action grab = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.GROUND);
            hand.grab(1000);
            return false;
        };
        Action raiseShoulder = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(1.0, shoulder.getPositionForMode(Shoulder.Mode.SEARCH , arm.getCurrentPosition()) * 3 / 4);
            hand.hangSample();
            return false;
        };
        Action snag = new SequentialAction(
                new CompleteAction(grab, hand),
                new CompleteAction(raiseShoulder, shoulder));
        Actions.runBlocking(snag);
    }
    private boolean search = false;
    private boolean hang = false;

    @Override
    public void run() {

        boolean pressingX = false;
        boolean pressingB = false;

        while (!isInterrupted()) {

            if (gamepad.x && !pressingX) {
                hand.hangSample();
                pressingX = true;
                if(shoulder.getMode() == Shoulder.Mode.SEARCH)
                    search = true;
            }
            else if(!gamepad.x){
                if(pressingX) {
                    pressingX = false;
                }
            }
            if (gamepad.b && !pressingB) {
                pressingB = true;
                if(shoulder.getMode() == Shoulder.Mode.HIGH_BAR)
                    hang = true;
            }
            else if(!gamepad.b){
                if(pressingB) {
                    pressingB = false;
                }
            }

            /*
            if(block == null)
                lights.setPosition(OFF_POWER);
            else if(block.id == YELLOW_ID)
                lights.setPosition(YELLOW_POWER);
            else if(block.id == RED_ID)
                lights.setPosition(RED_POWER);
            else if(block.id == BLUE_ID)
                lights.setPosition(BLUE_POWER);
            else
                lights.setPosition(OFF_POWER);
             */

            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }

    @Override
    public void safeHold(int position) {

    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

     class Pipeline extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            if(search){
                search = false;
                input = fp.matToDetection(input, color, favorYellow);

                // Stats
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

                // Find closest center and rotate to angle
                if(fp.centerXVal.size() > 0){
                     smallestDist = 10000;
                     smallestDistIndex = 0;
                    // use pythagorean theorem to draw a line from the camera to each block's position
                    // whichever block has the shortest distance is the block we are closest to
                    for (int i = 0; i < fp.centerXVal.size(); i++) {
                        // get the distance of the block from the center of the screen
                        int xDiff = Math.abs(TARGET_X - fp.centerXVal.get(i));
                        int yDiff = Math.abs(TARGET_Y - fp.centerYVal.get(i));

                        double dist = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
                        if (dist < smallestDist) {
                            smallestDist = dist;
                            smallestDistIndex = i;
                        }
                    }

                    double wristPos = fp.angleVal.get(smallestDistIndex) / Math.PI;
                    hand.setWrist(wristPos);
                    moveToColor();


                    // Move arm for a good pickup
                    //moveArmToColor();
                    //moveLegsToColor();
                    // Move wrist with a good average
                    //double wristPos = fp.angleVal.get(smallestDistIndex) / Math.PI;
                    //hand.setWrist(wristPos);

                    //plunge();

                    // Debug
                    /*
                    telemetry.addData("Total num", fp.centerXVal.size());
                    telemetry.addData("Winner x", fp.centerXVal.get(smallestDistIndex));
                    telemetry.addData("Winner y", fp.centerYVal.get(smallestDistIndex));
                    telemetry.addData("Winner a", Math.toDegrees(fp.angleVal.get(smallestDistIndex)));
                    telemetry.addData("Wrist pos", wristPos);

                     */
                }
                //telemetry.update();
            }
            if(hang){
                // Only check once
                hang = false;
                input = fp.matToBar(input, color);

                // Stats
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

                // Act on the y positions
                final int EXPECTED_LEFT_Y = 237;
                final int EXPECTED_RIGHT_Y = 242;
                final int TOO_CLOSE_LEFT_Y = 209;
                final int TOO_CLOSE_RIGHT_Y = 213;
                final int TOO_FAR_LEFT_Y = 308;
                final int TOO_FAR_RIGHT_Y = 315;
                final double TOO_CLOSE_DISTANCE_IN = 1.0;
                final double TOO_FAR_DISTANCE_IN = 3.0;
                final double IN_PER_PIXEL_TOO_CLOSE_LEFT = TOO_CLOSE_DISTANCE_IN / (double)(EXPECTED_LEFT_Y - TOO_CLOSE_LEFT_Y);
                final double IN_PER_PIXEL_TOO_FAR_LEFT = TOO_FAR_DISTANCE_IN / (double)(TOO_FAR_LEFT_Y - EXPECTED_LEFT_Y);
                final double IN_PER_PIXEL_TOO_CLOSE_RIGHT = TOO_CLOSE_DISTANCE_IN / (double)(EXPECTED_RIGHT_Y - TOO_CLOSE_RIGHT_Y);
                final double IN_PER_PIXEL_TOO_FAR_RIGHT = TOO_FAR_DISTANCE_IN / (double)(TOO_FAR_RIGHT_Y - EXPECTED_RIGHT_Y);

                if(fp.bar_left_y > 0 && fp.bar_right_y > 0){
                    int deltaLeft = fp.bar_left_y - EXPECTED_LEFT_Y;
                    int deltaRight = fp.bar_right_y - EXPECTED_RIGHT_Y;

                    inchesLeft = deltaLeft * ((deltaLeft < 0) ? IN_PER_PIXEL_TOO_CLOSE_LEFT : IN_PER_PIXEL_TOO_FAR_LEFT);
                    inchesRight = deltaRight * ((deltaRight < 0) ? IN_PER_PIXEL_TOO_CLOSE_RIGHT : IN_PER_PIXEL_TOO_FAR_RIGHT);

                    // Wiggle robot
                    moveLegsToBar(inchesLeft, inchesRight);

                    // Debug
                    //telemetry.addData("Inches left", inchesLeft);
                    //telemetry.addData("Inches right", inchesRight);
                }
                //telemetry.update();
            }
            return input;
        }
    }
}
