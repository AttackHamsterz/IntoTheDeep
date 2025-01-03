package org.firstinspires.ftc.teamcode;

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

    protected static final int WEBCAM_WIDTH = 1920;
    protected static final int WEBCAM_HEIGHT = 1080;
    public static final int TARGET_X = WEBCAM_WIDTH / 2;
    public static final int TARGET_Y = WEBCAM_HEIGHT * 4 / 10;

    // Arm calibration values
    protected static final int NEAR_Y = 186;
    protected static final int FAR_Y = 66;
    protected static final int NEAR_TICKS = 0;
    protected static final int FAR_TICKS = 680;

    // y=mx+b where y is ticks and x is the relative y pixel location
    protected static final double M = (double) (FAR_TICKS - NEAR_TICKS) / (double) (FAR_Y - NEAR_Y);
    protected static final double B = (double) FAR_TICKS - (M * (double) FAR_Y);

    // Leg calibration values
    protected static final int CENTER_X = 160;
    protected static final int INCHES_FROM_CENTER = 4;
    protected static final int SHIFT_NEAR_X = 283;
    protected static final int SHIFT_NEAR_Y = 139;
    protected static final double SHIFT_NEAR_M = (double) INCHES_FROM_CENTER / (double) (CENTER_X - SHIFT_NEAR_X);

    protected static final int SHIFT_FAR_X = 252;
    protected static final int SHIFT_FAR_Y = 109;
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

    public Eye(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color, boolean favorYellow, MecanumDrive legs, Arm arm, Shoulder shoulder, Hand hand, Gamepad gamepad, Telemetry telemetry) {
        this.legs = legs;
        this.arm = arm;
        this.shoulder = shoulder;
        this.hand = hand;
        this.color = color;
        this.favorYellow = favorYellow;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

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

    public void moveArmToColor() {
        camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block block = camera.getClosestBlock();
        if (block != null) {
            // Set new arm position!
            int ticks = (int) Math.round((M * (double) block.y + B));
            telemetry.addData("M", M);
            telemetry.addData("blockCenterY", block.y);
            telemetry.addData("B", B);
            telemetry.addData("deltaTicks", ticks);
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            arm.setPosition(0.3, arm.getCurrentPosition() + ticks);
        }
    }

    public void moveLegsToColor() {

    }
    private boolean search = false;
    private boolean hang = false;

    @Override
    public void run() {

        boolean pressingX = false;
        while (!isInterrupted()) {
            if (gamepad.x && !pressingX) {
                pressingX = true;
                search = true;
            }
            else{
                pressingX = false;
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

        // start the color recognition algorithm
        //camera.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void safeHold(int position) {

    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

     class Pipeline extends OpenCvPipeline {
        FrameProcessing fp = new FrameProcessing(WEBCAM_WIDTH, WEBCAM_HEIGHT);

        @Override
        public Mat processFrame(Mat input) {
            if(search){
                search = false;
                fp.matToDetection(input, color, favorYellow);

                // Stats
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

                // Find closest center and rotate to angle
                if(fp.centerXVal.size() > 0){
                    double smallestDist = 10000;
                    int smallestDistIndex = 0;
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

                    // Move body for a good pickup

                    // Move wrist with a good average
                    double wristPos = 0.5 - Math.cos(fp.angleVal.get(smallestDistIndex)) * 0.5;
                    hand.setWrist(wristPos);

                    // Plunge to pickup

                    // Debug
                    telemetry.addData("Total num", fp.centerXVal.size());
                    telemetry.addData("Winner x", fp.centerXVal.get(smallestDistIndex));
                    telemetry.addData("Winner y", fp.centerYVal.get(smallestDistIndex));
                    telemetry.addData("Winner a", Math.toDegrees(fp.angleVal.get(smallestDistIndex)));
                    telemetry.addData("Wrist pos", wristPos);
                }
                telemetry.update();
            }
            if(hang){
                hang = false;
                fp.matToBar(input);
            }
            return input;
        }
    }
}
