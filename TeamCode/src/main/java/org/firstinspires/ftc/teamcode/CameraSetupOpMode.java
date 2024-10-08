package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Robot Setup Camera Super Class", group = "Robot")
@Disabled
public class CameraSetupOpMode extends LinearOpMode {

    // April Tag Setup (for 1080p camera)
    static final int IMAGE_WIDTH = 800;     // Camera can do up to 1920x1080
    static final int IMAGE_HEIGHT = 448;    // Camera can do up to 1920x1080
    static final double TAG_SIZE = 0.0508;  // 2in tag
    static final double FX = 578.272;       // Calibration for 800x448
    static final double FY = 578.272;       // Calibration for 800x448
    static final double CX = 402.145;       // Calibration for 800x448
    static final double CY = 221.506;       // Calibration for 800x448
    static final double INCHES_PER_METER = 39.3701;

    private boolean set = false;  // If this is false we continually look for the hamster
    private boolean pose = false; // If this is true we make one april tag measurement

    protected OpenCvWebcam webcam = null;

    //Badger Bots ideal values
    static final Scalar RED = new Scalar(140, 23, 21);
    static final Scalar BLUE = new Scalar(5, 39, 80);
    static final Scalar GREY = new Scalar(140,140,140);
    static final int RED_THRESH = 50;
    static final int BLUE_THRESH = 50;

    static final int RED_REGION1_CAL = 0;
    static final int RED_REGION2_CAL = 0;
    static final int RED_REGION3_CAL = 0;
    static final int BLUE_REGION1_CAL = 0;
    static final int BLUE_REGION2_CAL = 0;
    static final int BLUE_REGION3_CAL = 0;

    public boolean red = false;

    /**
     * Method to stop the internal openCV loop from continuously detecting the hamster.
     */
    public void set(){
        set = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Build the camera class
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new CameraCalibration());
        //webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                //Camera Starts Running
                webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //Run this Code if there is an Error
            }
        });
    }

    /**
     * OpenCvPielne does two things:
     *  1) Detects the hamster and determines which side it is on
     *  2) Detects an april tag and positions the robot in front of it
     */
    public class CameraCalibration extends OpenCvPipeline {
        private void colorDetermination(Mat input, AtomicInteger redCount, AtomicInteger blueCount) {
            int redTot = 0;
            int blueTot = 0;

            double redDist = 0;
            double blueDist = 0;

            //Goes through every pixel of input
            for (int r = 0; r < input.rows(); r++) {
                for (int c = 0; c < input.cols(); c++) {
                    // Get pixel RGB value
                    double[] v = input.get(r, c);

                    // Get distances
                    redDist = Math.sqrt(Math.pow((v[0] - RED.val[0]), 2) + Math.pow((v[1] - RED.val[1]), 2) + Math.pow((v[2] - RED.val[2]), 2));
                    blueDist = Math.sqrt(Math.pow((v[0] - BLUE.val[0]), 2) + Math.pow((v[1] - BLUE.val[1]), 2) + Math.pow((v[2] - BLUE.val[2]), 2));

                    // Increment counts if within threshold
                    if (redDist < blueDist) {
                        if (redDist < RED_THRESH) {
                            if (red) {
                                redTot++;
                            }
                        }
                    } else {
                        if (blueDist < BLUE_THRESH) {
                            if (!red) {
                                blueTot++;
                            }
                        }
                    }
                }
            }
            redCount.set(redTot);
            blueCount.set(blueTot);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Should we detect an april tag and then pose the robot in front of it?
            if(pose) {
                // Detect AprilTag
                Mat grey = new Mat();
                Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

                // Only detect and move the robot once
                // TODO - could run twice!
                pose = false;
            }

            // Have we already detected the hamster? (yes - then exit, no continue)
            if(set)return input;

            // Dynamic sizing
            Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,IMAGE_HEIGHT / 3);
            Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(IMAGE_WIDTH / 3,IMAGE_HEIGHT / 3);
            Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(IMAGE_WIDTH / 3 * 2,IMAGE_HEIGHT / 3);
            int REGION_WIDTH = IMAGE_WIDTH / 3;
            int REGION_HEIGHT = IMAGE_HEIGHT - IMAGE_HEIGHT /3;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            Point region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            Point region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            //Dividing our region into three sections using our points above
            Mat region1 = input.submat(new Rect(region1_pointA, region1_pointB));
            Mat region2 = input.submat(new Rect(region2_pointA, region2_pointB));
            Mat region3= input.submat(new Rect(region3_pointA, region3_pointB));

            //A test region to find the color values of an object
            //Mat test = input.submat(new Rect(160, 160, 10,10));

            //Gets the average of each color inside the test area
            //For finding the color value of objects
            //Scalar mean = Core.mean(test);

            //Creating vars to store the red and blue values for each of these regions
            AtomicInteger red1 = new AtomicInteger();
            AtomicInteger blue1 = new AtomicInteger();
            AtomicInteger red2 = new AtomicInteger();
            AtomicInteger blue2 = new AtomicInteger();
            AtomicInteger red3 = new AtomicInteger();
            AtomicInteger blue3 = new AtomicInteger();

            colorDetermination(region1, red1, blue1);
            colorDetermination(region2, red2, blue2);
            colorDetermination(region3, red3, blue3);

            // Remove calibration values from counts
            red1.addAndGet(-RED_REGION1_CAL);
            red2.addAndGet(-RED_REGION2_CAL);
            red3.addAndGet(-RED_REGION3_CAL);
            blue1.addAndGet(-BLUE_REGION1_CAL);
            blue2.addAndGet(-BLUE_REGION2_CAL);
            blue3.addAndGet(-BLUE_REGION3_CAL);

            //Use Math.max to determine if it is red or blue
            //Use nested if statements to determine which region it is in
            String colors;
            String region;
            int maxRed = Math.max(red1.get(), Math.max(red2.get(), red3.get()));
            int maxBlue = Math.max(blue1.get(), Math.max(blue2.get(), blue3.get()));
            if (maxRed > maxBlue) {
                //It is red

            } else {
                //It is blue

            }

            //Displays rectangles for lining up the webcam
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //webcam.closeCameraDevice();
            /*
            Imgproc.rectangle(
                    input,
                    new Point(160, 160),
                    new Point(170, 170),
                    new Scalar(0,0,0),
                    1);

             */

            //  Debug
            telemetry.addData("Red 1", red1.get());
            telemetry.addData("Red 2", red2.get());
            telemetry.addData("Red 3", red3.get());
            telemetry.addData("Blue 1", blue1.get());
            telemetry.addData("Blue 2", blue2.get());
            telemetry.addData("Blue 3", blue3.get());
            //telemetry.addData("Mean", "%d %d %d", (int) mean.val[0], (int) mean.val[1], (int) mean.val[2]);
            telemetry.update();
            return input;
        }
    }
}