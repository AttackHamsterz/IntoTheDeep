package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


public class Eye extends BodyPart {

    private static final boolean ENABLE_DASHBOARD_CAMERA = false;
    private static final double BLACK_COLOR = 0.0;
    private static final double RED_COLOR = 0.279;
    private static final double YELLOW_COLOR = 0.388;
    private static final double BLUE_COLOR = 0.611;
    private static final double WHITE_COLOR = 1.0;

    // variable to store the color of alliance
    public static final int NONE_ID = 0;
    public static final int YELLOW_ID = 1;
    public static final int RED_ID = 2;
    public static final int BLUE_ID = 3;
    private int grabColor;
    private int searchColor;
    private boolean fullMove = false;

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

    // Bar calibration values
    //protected static final int EXPECTED_LEFT_Y = 88;//86;
    //protected static final int EXPECTED_RIGHT_Y = 96;//92;
    //protected static final int TOO_CLOSE_LEFT_Y = 60;//55;
    //protected static final int TOO_CLOSE_RIGHT_Y = 68;//62;
    //protected static final int TOO_FAR_LEFT_Y = 144;//141;
    //protected static final int TOO_FAR_RIGHT_Y = 152;//151;
    protected static final int EXPECTED_Y = 90;
    protected static final int TOO_CLOSE_Y = 62;
    protected static final int TOO_FAR_Y = 145;
    protected static final double TOO_CLOSE_DISTANCE_IN = 1.25;
    protected static final double TOO_FAR_DISTANCE_IN = 4.0;

    // values for the plane
    CalibrationPoint[] calibrationPlane = {
            new CalibrationPoint(140, 445, 1250, 4), // top left
            new CalibrationPoint(116, 505, 370, 4), // bottom left
            new CalibrationPoint(352, 501, 370, -4), // bottom right
            new CalibrationPoint(311, 445, 1250, -4), // top right
            new CalibrationPoint(232, 557, 0, 0), // center
            new CalibrationPoint(224, 449, 1250,0) // far center
    };

    PlaneFit plane = new PlaneFit(Arrays.asList(calibrationPlane));

    OpenCvWebcam webcam;

    StandardSetupOpMode.COLOR color;
    boolean favorYellow;
    public Servo grabLight;
    public Servo searchLight;
    private final FrameProcessing fp;
    private int smallestDistIndex = 0;
    public double inches;
    //public double inchesLeft;
    //public double inchesRight;
    public Action barAction;
    public Action searchAction;
    public Action floorAction;
    public boolean search = false;
    private boolean hang = false;
    private boolean floor = false;
    private double maxSearchDist;
    private double maxHangDist;
    private double maxFloorDist;
    public boolean searching;
    private boolean testGrey = false;


    public Eye(StandardSetupOpMode ssom){
        super.setStandardSetupOpMode(ssom);
        this.color = ssom.color;
        this.favorYellow = ssom.favorYellow;
        this.gamepad = ssom.gamepad2;
        this.extraGamepad = ssom.gamepad1;
        this.searching = false;

        fp = new FrameProcessing(WEBCAM_WIDTH, WEBCAM_HEIGHT, ssom.telemetry);

        // LED setup
        this.grabLight = ssom.hardwareMap.get(Servo.class, "grabLight"); // Expansion hub ch3
        this.searchLight = ssom.hardwareMap.get(Servo.class, "searchLight"); // Control hub ch3
        grabColor = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;
        searchColor = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;
        if(grabColor == BLUE_ID) {
            grabLight.setPosition(BLUE_COLOR);
        }
        else {
            grabLight.setPosition(RED_COLOR);
        }

        int cameraMonitorViewId = ssom.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ssom.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(ssom.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if(ENABLE_DASHBOARD_CAMERA) {
                    FtcDashboard.getInstance().startCameraStream(webcam, 60);
                }
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                if(ssom.telemetry != null) {
                    ssom.telemetry.addLine("Camera failed to init");
                    ssom.telemetry.update();
                }
            }
        });
    }

    public void debugTelemetry(Telemetry telemetry)
    {
        // Webcam debug
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        //telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        //telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        //telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        //telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

        // Contour detection debug
        if (fp.centerXVal.size() > 0) {
            telemetry.addData("Total samples", fp.centerXVal.size());
            telemetry.addData("Winner x", fp.centerXVal.get(smallestDistIndex));
            telemetry.addData("Winner y", fp.centerYVal.get(smallestDistIndex));
            telemetry.addData("Winner a", Math.toDegrees(fp.angleVal.get(smallestDistIndex)));
        }

        // Bar detection debug
        if(fp.bar_y > 0) {
            telemetry.addData("Inches", inches);
            telemetry.addData("bar y", fp.bar_y);
        }
        //if(fp.bar_left_y > 0 && fp.bar_right_y > 0) {
        //    telemetry.addData("Inches left", inchesLeft);
        //    telemetry.addData("Inches right", inchesRight);
        //    telemetry.addData("left bar y", fp.bar_left_y);
        //    telemetry.addData("right bar y", fp.bar_right_y);
        //}


        if(fp.floor_left != 0 || fp.floor_forward != 0){
            telemetry.addData("Floor left", fp.floor_left);
            telemetry.addData("Floor forward", fp.floor_forward);
            telemetry.addData("Cx", fp.cx);
            telemetry.addData("Cy", fp.cy);
        }
    }

    public void lightOn(){
        searchLight.setPosition(WHITE_COLOR);
    }

    public void lightOff(){
        searchLight.setPosition(BLACK_COLOR);
    }

    public Action moveToColor() {
        Action moveAction;

        if(!fp.centerXVal.isEmpty()) {
            double smallestDist = 10000;
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
            ssom.hand.setWrist(wristPos);

            if(fullMove) {
                // Set new arm position!
                Action moveArm = telemetryPacket -> {
                    int ticks = plane.getTicks(fp.centerXVal.get(smallestDistIndex), fp.centerYVal.get(smallestDistIndex));
                    ssom.arm.setPosition(1.0, ssom.arm.getCurrentPosition() + ticks);
                    return false;
                };
                Action strafeToBlock = telemetryPacket -> {
                    double shift = plane.getShift(fp.centerXVal.get(smallestDistIndex), fp.centerYVal.get(smallestDistIndex));
                    ssom.legs.moveLeft(shift, false);
                    return false;
                };
                Action grab = telemetryPacket -> {
                    ssom.shoulder.setPositionForMode(Shoulder.Mode.GROUND, 1.0, ssom.arm.getCurrentPosition());
                    ssom.hand.grab(1000);
                    return false;
                };
                Action raiseShoulder = telemetryPacket -> {
                    ssom.shoulder.setMode(Shoulder.Mode.NONE);
                    ssom.shoulder.setPosition(0.8, ssom.shoulder.getPositionForMode(Shoulder.Mode.SEARCH, ssom.arm.getCurrentPosition()) * 3 / 4);
                    //hand.hangSample();
                    return false;
                };

                ParallelAction premove = new ParallelAction(
                        new CompleteAction(moveArm, ssom.arm),
                        new CompleteAction(strafeToBlock, ssom.legs)
                );
                moveAction = new SequentialAction(
                        premove,
                        new CompleteAction(grab, ssom.hand),
                        new CompleteAction(raiseShoulder, ssom.shoulder));
            }
            else{
                moveAction = telemetryPacket -> {
                    return false;
                };
            }
        }
        else {
            moveAction = telemetryPacket -> {
                return false;
            };
        }
        return moveAction;
    }

    public int getGrabColor(){
        return grabColor;
    }

    public void moveLegsToColor() {
        Action strafeToBlock = telemetryPacket -> {
            double ySlope = fp.centerYVal.get(smallestDistIndex) * SHIFT_M + SHIFT_B;
            double shift = (fp.centerXVal.get(smallestDistIndex) - CENTER_X) * ySlope;
            ssom.legs.moveLeft(shift, false);
            return false;
        };
        SequentialAction centerBlockAction = new SequentialAction(
                new CompleteAction(strafeToBlock, ssom.legs));
        Actions.runBlocking(centerBlockAction);
    }

    public Action moveLegsToBar(double leftInches, double rightInches){
        ssom.shoulder.setMode(Shoulder.Mode.NONE);
        Action moveToBar = telemetryPacket -> {
            double averageInches = (leftInches + rightInches) / 2.0;
            ssom.legs.moveForward(averageInches, false);
            return false;
        };
        //Action lowerShoulder = telemetryPacket -> {
        //    ssom.shoulder.setPosition(1.0, 1143);
        //    return false;
        //};
        //SequentialAction gotoBar = new SequentialAction(
        //        new CompleteAction(moveToBar, ssom.legs, 700));
        //        new CompleteAction(lowerShoulder, ssom.shoulder, 700);

        return new CompleteAction(moveToBar, ssom.legs, 750);
    }

    public void plunge() {
        Action grab = telemetryPacket -> {
            ssom.shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.8, ssom.arm.getCurrentPosition());
            ssom.hand.grab(1000);
            return false;
        };
        Action raiseShoulder = telemetryPacket -> {
            ssom.shoulder.setMode(Shoulder.Mode.NONE);
            ssom.shoulder.setPosition(1.0, ssom.shoulder.getPositionForMode(Shoulder.Mode.SEARCH , ssom.arm.getCurrentPosition()) * 3 / 4);
            ssom.hand.hangSample();
            return false;
        };
        Action snag = new SequentialAction(
                new CompleteAction(grab, ssom.hand),
                new CompleteAction(raiseShoulder, ssom.shoulder));
        Actions.runBlocking(snag);
    }

    @Override
    public void run() {
        boolean pressing = false;
        while (!isInterrupted()) {

            if(!ignoreGamepad) {
                if(!pressing && extraGamepad.back){
                    pressing = true;
                    fullMove = !fullMove;
                }
                if(!extraGamepad.back)
                    pressing = false;

                /*
                if (gamepad.b) {
                    testGrey = true;
                }

                 */
            }

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

    /**
     * Tell the camera to check the bar and return an action to align to it
     * @return Action that aligns to the bar
     */
    public Action safeHang(double maxDist) {
        // Wait for robot to stop moving
        /*
        try {
            sleep(200);
        } catch (InterruptedException ignored) {
        }

         */

        // Tell the opencv thread to get an answer and then wait for result
        maxHangDist = Math.abs(maxDist);
        hang = true;

        // Spin wait until we get a result
        do {
            // Give the pipeline a little time to build a result
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException ignored) {
            }
        }while(barAction == null);

        return barAction;
    }

    /**
     * Tell the camera to check the bar and return an action to align to it
     * @return Action that aligns to the bar
     */
    public Action safeSearch(double maxDist) {
        // Tell the opencv thread to get an answer and then wait for result
        maxSearchDist = Math.abs(maxDist);
        search = true;

        // Spin wait until we get a result
        do {
            // Give the pipeline a little time to build a result
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException ignored) {
            }
        }while(searchAction == null);

        return searchAction;
    }

    /**
     * Tell the camera to check the floor and return an action to align to specimen
     * @return Action that aligns to the specimen
     */
    public Action safeFloor(double maxDist){
        // Wait for robot to stop moving
        //try {
        //    sleep(200);
        //} catch (InterruptedException ignored) {
        //}

        // Tell the opencv thread to get an answer and then wait for the result
        maxFloorDist = Math.abs(maxDist);
        floor = true;

        // Spin wait until we get a result
        do {
            // Give the pipeline a little time to build a result
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException ignored) {
            }
        }while(floorAction == null);

        return floorAction;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    static final int EVERY_NTH_FRAME = 25;

     class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            // For every nth frame, lets see what we're holding
            int frameCount = webcam.getFrameCount();

            if(frameCount % EVERY_NTH_FRAME == 0){
                // What color did I grab (only check if hand position is close to centered)
                // Also only check if arm is fully in and we are in search mode
                double wp = ssom.hand.getWristPosition();
                if(wp > Hand.CTR_POS - 0.1
                        && wp < Hand.CTR_POS + 0.1
                        && Math.abs(ssom.shoulder.getCurrentPosition() - ssom.shoulder.getPositionForMode(Shoulder.Mode.SEARCH, 0)) <= 50
                        && ssom.arm.getCurrentPosition() <= 50) {
                    //grabColor = fp.grabColor(input);
                    grabColor = fp.checkHoldingColor(input);
                    if (grabColor == YELLOW_ID)
                        grabLight.setPosition(YELLOW_COLOR);
                    else if (grabColor == RED_ID)
                        grabLight.setPosition(RED_COLOR);
                    else if (grabColor == BLUE_ID)
                        grabLight.setPosition(BLUE_COLOR);
                    else
                        grabLight.setPosition(BLACK_COLOR);
                }
                else
                    grabLight.setPosition(BLACK_COLOR);

                // What color did I search and find
                /*
                switch (fp.colorID){

                    case YELLOW_ID:
                        searchLight.setPosition(YELLOW_POWER);
                        break;
                    case BLUE_ID:
                        searchLight.setPosition(BLUE_POWER);
                        break;
                    case RED_ID:
                        searchLight.setPosition(RED_POWER);
                        break;
                    default:
                        searchLight.setPosition(BLACK_POWER);
                };

                 */
            }

            if(search){

                searching = true;
                searchAction = null;
                search = false;
                //ssom.shoulder.setMode(Shoulder.Mode.SEARCH);
                input = fp.matToDetection(input, color, favorYellow);

                // Build search action (blocking)
                searchAction = moveToColor();
                searching = false;


            }
            if(hang){
                // Only check once
                barAction = null;
                hang = false;
                input = fp.matToBar(input, color);

                // Act on the y positions
                final double IN_PER_PIXEL_TOO_CLOSE = TOO_CLOSE_DISTANCE_IN / (double)(EXPECTED_Y - TOO_CLOSE_Y);
                final double IN_PER_PIXEL_TOO_FAR = TOO_FAR_DISTANCE_IN / (double)(TOO_FAR_Y - EXPECTED_Y);
                //final double IN_PER_PIXEL_TOO_CLOSE_LEFT = TOO_CLOSE_DISTANCE_IN / (double)(EXPECTED_LEFT_Y - TOO_CLOSE_LEFT_Y);
                //final double IN_PER_PIXEL_TOO_FAR_LEFT = TOO_FAR_DISTANCE_IN / (double)(TOO_FAR_LEFT_Y - EXPECTED_LEFT_Y);
                //final double IN_PER_PIXEL_TOO_CLOSE_RIGHT = TOO_CLOSE_DISTANCE_IN / (double)(EXPECTED_RIGHT_Y - TOO_CLOSE_RIGHT_Y);
                //final double IN_PER_PIXEL_TOO_FAR_RIGHT = TOO_FAR_DISTANCE_IN / (double)(TOO_FAR_RIGHT_Y - EXPECTED_RIGHT_Y);

                //int deltaLeft = (fp.bar_left_y > 0) ? fp.bar_left_y - EXPECTED_LEFT_Y : 0;
                //int deltaRight = (fp.bar_right_y > 0) ? fp.bar_right_y - EXPECTED_RIGHT_Y : 0;
                int delta = (fp.bar_y > 0) ? fp.bar_y - EXPECTED_Y : 0;

                //if(fp.bar_left_y < 0) deltaLeft = deltaRight;
                //if(fp.bar_right_y < 0) deltaRight = deltaLeft;

                //if(Math.abs(deltaLeft) > 0.1 || Math.abs(deltaRight) > 0.1){
                if(Math.abs(delta) > 0.1){
                    // Wiggle robot (never let it back up too far)
                    inches = Range.clip((double)delta * ((delta < 0) ? IN_PER_PIXEL_TOO_CLOSE : IN_PER_PIXEL_TOO_FAR), -0.75, maxHangDist);
                    barAction = moveLegsToBar(inches, inches);
                    //inchesLeft = Range.clip((double)deltaLeft * ((deltaLeft < 0) ? IN_PER_PIXEL_TOO_CLOSE_LEFT : IN_PER_PIXEL_TOO_FAR_LEFT), -0.5, maxHangDist);
                    //inchesRight = Range.clip((double)deltaRight * ((deltaRight < 0) ? IN_PER_PIXEL_TOO_CLOSE_RIGHT : IN_PER_PIXEL_TOO_FAR_RIGHT), -0.5, maxHangDist);
                    //barAction = moveLegsToBar(inchesLeft, inchesRight);
                }
                else {
                        barAction = telemetryPacket -> {
                            return false;
                        };
                }
            }

            if(floor){

                // Check only once
                floorAction = null;
                floor = false;
                input = fp.matToFloor(input, color);

                // Act on the positions
                if(Math.abs(fp.floor_left) > 0.1 || Math.abs(fp.floor_forward) > 0.1){

                    // Wiggle robot
                    double inches_left = Range.clip(fp.floor_left,-maxFloorDist, maxFloorDist);
                    double inches_forward = Range.clip(fp.floor_forward,-maxFloorDist, maxFloorDist);
                    Action moveToSpecimen = telemetryPacket -> {
                        ssom.legs.moveForwardAndLeft(inches_forward, inches_left, false);
                        return false;
                    };
                    // Give yourselves half a second per inch
                    floorAction = new CompleteAction(moveToSpecimen, ssom.legs, Math.round(500* maxFloorDist));
                }
                else {
                    floorAction = telemetryPacket -> {
                        return false;
                    };
                }



            }

            /*
            if (testGrey) {
                input = fp.greyTesting(input);
            }

             */


            // Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            return input;
        }
    }
}
