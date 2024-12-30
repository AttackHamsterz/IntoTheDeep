package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Eye extends BodyPart {

    private static final double OFF_POWER = 0.0;
    private static final double RED_POWER = 0.279;
    private static final double YELLOW_POWER = 0.388;
    private static final double BLUE_POWER = 0.555;

    // variable to store the color of alliance
    private static final int NONE_ID = 0;
    private static final int YELLOW_ID = 1;
    private static final int RED_ID = 2;
    private static final int BLUE_ID = 3;
    private final int colorId;

    protected static final int WEBCAM_WIDTH = 640;
    protected static final int WEBCAM_HEIGHT = 480;
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

    private ArrayList<Double> winnerAngles = new ArrayList<>();

    ColorCamera camera;
    OpenCvWebcam webcam;
    Shoulder shoulder;
    Arm arm;
    Gamepad gamepad;
    StandardSetupOpMode.COLOR color;
    Telemetry telemetry;
    public Servo lights;

    public Eye(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color, Shoulder shoulder, Arm arm, Gamepad gamepad, Telemetry telemetry) {
        this.shoulder = shoulder;
        this.arm = arm;
        //this.color = color;
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
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

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

    @Override
    public void run() {


        while (!isInterrupted()) {


            if (gamepad.x) {
                // start the webcam
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Winner Angles", winnerAngles);
            telemetry.update();

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
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Mat lab = new Mat();

            Imgproc.cvtColor(input, lab, Imgproc.COLOR_BGR2Lab);

            Scalar yellow_low = new Scalar(220, 110, 150);
            Scalar yellow_high = new Scalar(255, 125, 220);
            Mat yellowMask = new Mat();
            Core.inRange(lab, yellow_low, yellow_high, yellowMask);

            Scalar blue_low = new Scalar(70, 140, 45);
            Scalar blue_high = new Scalar(150, 170, 65);
            Mat blueMask = new Mat();
            Core.inRange(lab, blue_low, blue_high, blueMask);

            Scalar red_low = new Scalar(160, 140, 125);
            Scalar red_high = new Scalar(240, 190, 190);
            Mat redMask = new Mat();
            Core.inRange(lab, red_low, red_high, redMask);




            Mat mask = Mat.zeros(lab.size(), CvType.CV_8UC1); // Create an empty mask
            // Set values for yellow, red, and blue masks in the combined mask
            Mat nonZeroYellowMask = new Mat();
            Core.compare(yellowMask, new Scalar(0), nonZeroYellowMask, Core.CMP_NE);
            mask.setTo(new Scalar(255), nonZeroYellowMask); // Set yellow to 255

            Mat nonZeroRedMaskFinal = new Mat();
            Core.compare(redMask, new Scalar(0), nonZeroRedMaskFinal, Core.CMP_NE);
            mask.setTo(new Scalar(85), nonZeroRedMaskFinal); // Set red to 85

            Mat nonZeroBlueMaskFinal = new Mat();
            Core.compare(blueMask, new Scalar(0), nonZeroBlueMaskFinal, Core.CMP_NE);
            mask.setTo(new Scalar(170), nonZeroBlueMaskFinal); // Set blue to 170
            // Step 9: Find contours
            List<MatOfPoint> yellowContours = new java.util.ArrayList<>();
            List<MatOfPoint> redContours = new java.util.ArrayList<>();
            List<MatOfPoint> blueContours = new java.util.ArrayList<>();
            // Find contours for each color mask
            Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(redMask, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            telemetry.addData("yellow contours", yellowContours);
            telemetry.addData("red contours", redContours);
            telemetry.addData("blue contours", blueContours);

            int min_area = 10000;

            yellowContours.removeIf(cnt -> Imgproc.contourArea(cnt) <= min_area);
            redContours.removeIf(cnt -> Imgproc.contourArea(cnt) <= min_area);
            blueContours.removeIf(cnt -> Imgproc.contourArea(cnt) <= min_area);

            int binSize = 5;


/*
            telemetry.addData("yellow contours", yellowContours);
            telemetry.addData("red contours", redContours);
            telemetry.addData("blue contours", blueContours);

 */

            for (MatOfPoint contour : yellowContours) {
                // smooth contour
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double epsilon = 0.001 * Imgproc.arcLength(contour2f, true);
                MatOfPoint2f approxContour = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxContour, epsilon, true);

                int numBins = 180 / binSize;
                double[] hist = new double[numBins];
                Mat pointsMat = approxContour.reshape(-1, 2);
                Point[] points = new Point[pointsMat.rows()];
                for (int i = 0; i < pointsMat.rows(); i++) {
                    points[i] = new Point(pointsMat.get(i, 0));
                }

                ArrayList<Double> centerY = new ArrayList<>();
                for (int i = 1; i < points.length; i++) {
                    double midpointY = (points[i - 1].y + points[i].y) / 2;
                    centerY.add(midpointY);
                }

                ArrayList<Double> centerX = new ArrayList<>();
                for (int i = 1; i < points.length; i++) {
                    double midpointX = (points[i - 1].x + points[i].x) / 2;
                    centerX.add(midpointX);
                }


                ArrayList<Double> deltaY = new ArrayList<>();
                for (int i = 1; i < points.length; i++) {
                    double deltaYValue = points[i].y - points[i - 1].y;
                    deltaY.add(deltaYValue);
                }

                ArrayList<Double> deltaX = new ArrayList<>();
                for (int i = 1; i < points.length; i++) {
                    double deltaXValue = points[i].x - points[i - 1].x;
                    deltaX.add(deltaXValue);
                }


                ArrayList<Double> weights = new ArrayList<>();
                for (int i = 1; i < points.length; i++) {
                    double deltax = points[i].x - points[i - 1].x;
                    double deltay = points[i].y - points[i - 1].y;
                    double distance = Math.sqrt(deltay * deltay + deltax * deltax); // Euclidean distance
                    weights.add(distance);
                }
                // Normalize weights
                double sumWeights = 0;
                for (double weight : weights) {
                    sumWeights += weight;
                }
                for (int i = 0; i < weights.size(); i++) {
                    weights.set(i, weights.get(i) / sumWeights);
                }

                // Compute the weighted centroids
                double weightedCentery = 0;
                double weightedCenterx = 0;

                for (int i = 0; i < centerY.size(); i++) {
                    weightedCentery += centerY.get(i) * weights.get(i);
                    weightedCenterx += centerX.get(i) * weights.get(i);
                }

                int centerYVal = (int) weightedCentery;
                int centerXVal = (int) weightedCenterx;


                // Calculate angles
                double[] angles = new double[deltaY.size()];
                for (int i = 0; i < deltaX.size(); i++) {
                    angles[i] = Math.atan2(deltaY.get(i), deltaX.get(i));
                    if (angles[i] < 0) {
                        angles[i] += Math.PI;
                    }
                }

                // Normalize angles to histogram bins
                for (int i = 0; i < angles.length; i++) {
                    int idx = (int) Math.round((numBins - 1) * angles[i] / Math.PI);
                    hist[idx] += weights.get(i);
                }

                // Set histogram edges to 0
                hist[0] = 0;
                hist[numBins / 2] = 0;
                hist[numBins - 1] = 0;

                // Find the most prominent angle (winner)
                int winnerIdx = 0;
                double maxHistValue = hist[0];
                for (int i = 1; i < hist.length; i++) {
                    if (hist[i] > maxHistValue) {
                        maxHistValue = hist[i];
                        winnerIdx = i;
                    }
                }
                double winnerAngle = winnerIdx * Math.PI / (numBins - 1);

                winnerAngles.add(winnerAngle);
            }


            return input;
        }
    }
}
