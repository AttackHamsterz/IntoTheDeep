package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * A class to perform frame processing in opencv.  Also lets us easily unit test the algorithm
 */
public class FrameProcessing {
    private final Mat hsv;
    private final Mat mask;
    private final Mat maskLow;
    private final Mat maskHi;
    private final Mat hierarchy;
    private final Mat floorHierarchy;
    private final Mat image;


    public List<MatOfPoint> contours = new ArrayList<>();
    public List<Integer> centerXVal = new ArrayList<>();
    public List<Integer> centerYVal = new ArrayList<>();
    public List<Double> angleVal = new ArrayList<>();
    public int colorID = Eye.NONE_ID;

    private static final boolean DRAW = true;
    private final int MIN_AREA;

    private final Mat grab_slice;
    private final Mat grab_mask;

    private final Mat bar_slice;
    private final Mat bar_mask;
    private final Mat bar_low_mask;
    private final Mat bar_high_mask;

    //private final Mat bar_left_slice;
    //private final Mat bar_right_slice;
    //private final Mat bar_left_mask;
    //private final Mat bar_right_mask;

    public List<MatOfPoint> bar_contours = new ArrayList<>();
    //public List<MatOfPoint> bar_left_contours = new ArrayList<>();
    //public List<MatOfPoint> bar_right_contours = new ArrayList<>();
    public int bar_y;
    //public int bar_left_y;
    //public int bar_right_y;

    private final int BAR_SAMPLE_WIDTH = 40;
    private final int BAR_MIN_AREA = BAR_SAMPLE_WIDTH * 2;

    private static final double MAX_CHANGE = Math.toRadians(10);

    private static final Scalar HSV_YELLOW_LOW = new Scalar(20, 100, 100);
    private static final Scalar HSV_YELLOW_HIGH = new Scalar(40, 255, 255);
    private static final Scalar HSV_RED1_LOW = new Scalar(0, 100, 100);
    private static final Scalar HSV_RED1_HIGH = new Scalar(10, 255, 255);
    private static final Scalar HSV_RED2_LOW = new Scalar(160, 100, 100);
    private static final Scalar HSV_RED2_HIGH = new Scalar(180, 255, 255);
    private static final Scalar HSV_BLUE_LOW = new Scalar(100, 100, 100);
    private static final Scalar HSV_BLUE_HIGH = new Scalar(140, 255, 255);

    private Telemetry telemetry;

    // 320 - 400 width 80
    // 20 - 100
    private static final Rect GRAB_ROI = new Rect(320, 20, 80,100);
    private final Mat grab_yellow;
    private final Mat grab_redLow;
    private final Mat grab_redHigh;
    private final Mat grab_red;
    private final Mat grab_blue;


    // Floor calibration values (start with specimen, search height, arm in,
    // place specimen in ideal location, then back on motion controller to get centroid
    private static final int FLOOR_ALIGNED_X = 183;
    private static final int FLOOR_ALIGNED_Y = 128;

    // Shrinking these will cause smaller motion on detections
    private static final double IN_PER_PIXEL_LR = 0.025;
    private static final double IN_PER_PIXEL_FB = 0.11;

    public List<MatOfPoint> floor_contours = new ArrayList<>();
    private final Mat hsv_floor;
    private final Mat floor_low_mask;
    private final Mat floor_high_mask;
    private final Mat floor_mask;

    private final Mat hsv_grabbed;
    private final Mat grabbed_mask;
    private final Mat grabbed_low_mask;
    private final Mat grabbed_high_mask;
    private final Rect GRABBED_SAMPLE_ROI = new Rect(200, 360, 60, 60);

    public double floor_left;
    public double floor_forward;

    public int cx;
    public int cy;


    public FrameProcessing(int width, int height, Telemetry telemetry){
        hsv = new Mat(height, width * 3, CvType.CV_8U);
        mask = new Mat(height, width, CvType.CV_8UC1);
        maskLow = new Mat(height, width, CvType.CV_8UC1);
        maskHi = new Mat(height, width, CvType.CV_8UC1);
        hierarchy = new Mat();
        floorHierarchy = new Mat();
        if(DRAW)
            image = new Mat(height, width, CvType.CV_8U);
        MIN_AREA = (int)Math.round((double)(width * height) * 0.002);

        grab_slice = new Mat(40, 40, CvType.CV_8U);
        bar_slice = new Mat(height, BAR_SAMPLE_WIDTH * 3, CvType.CV_8U);
        //bar_left_slice = new Mat(height, BAR_SAMPLE_WIDTH * 3, CvType.CV_8U);
        //bar_right_slice = new Mat(height, BAR_SAMPLE_WIDTH * 3, CvType.CV_8U);

        grab_mask = new Mat(40, 40, CvType.CV_8UC1);
        bar_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        bar_low_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        bar_high_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        //bar_left_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        //bar_right_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);

        this.telemetry = telemetry;

        grab_yellow = new Mat(height, width, CvType.CV_8UC1);
        grab_redHigh = new Mat(height, width, CvType.CV_8UC1);
        grab_redLow = new Mat(height, width, CvType.CV_8UC1);
        grab_red = new Mat(height, width, CvType.CV_8UC1);
        grab_blue = new Mat(height, width, CvType.CV_8UC1);

        hsv_floor = new Mat(height, width * 3, CvType.CV_8U);
        floor_low_mask = new Mat(height, width, CvType.CV_8UC1);
        floor_high_mask = new Mat(height, width, CvType.CV_8UC1);
        floor_mask = new Mat(height, width, CvType.CV_8UC1);

        hsv_grabbed = new Mat(height, width, CvType.CV_8UC1);
        grabbed_mask = new Mat(height, width, CvType.CV_8UC1);
        grabbed_low_mask = new Mat(height, width, CvType.CV_8UC1);
        grabbed_high_mask = new Mat(height, width, CvType.CV_8UC1);
    }

    private static final int MIN_NUM_FOR_GRAB = 40;
    public int grabColor(Mat input ){

        Imgproc.cvtColor(input.submat(GRAB_ROI), grab_slice, Imgproc.COLOR_RGB2HSV);

        Core.inRange(grab_slice, HSV_YELLOW_LOW, HSV_YELLOW_HIGH, grab_yellow);
        Core.inRange(grab_slice, HSV_RED1_LOW, HSV_RED1_HIGH, grab_redLow);
        Core.inRange(grab_redLow, HSV_RED2_LOW, HSV_RED2_HIGH, grab_redHigh);
        Core.add(grab_redLow, grab_redHigh, grab_red);
        Core.inRange(grab_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, grab_blue);

        // Counts for each color
        int yellowAmount = Core.countNonZero(grab_yellow);
        int redAmount = Core.countNonZero(grab_red);
        int blueAmount = Core.countNonZero(grab_blue);

        // Ensure we have enough and it's not just parts of the tool
        if(yellowAmount<MIN_NUM_FOR_GRAB)
            yellowAmount = 0;
        if(redAmount<MIN_NUM_FOR_GRAB)
            redAmount = 0;
        if(blueAmount<MIN_NUM_FOR_GRAB)
            blueAmount = 0;

        // Return the correct color
        if (yellowAmount > redAmount && yellowAmount > blueAmount) {
            return Eye.YELLOW_ID;
        } else if (redAmount > blueAmount) {
            return Eye.RED_ID;
        } else if(blueAmount >= MIN_NUM_FOR_GRAB){
            return Eye.BLUE_ID;
        }
        else{
            return Eye.NONE_ID;
        }
    }

    public Mat matToDetection(Mat input, StandardSetupOpMode.COLOR alliance, boolean favorYellow)
    {
        // Convert from BGR to LAB
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Clear existing detections
        contours.clear();
        centerXVal.clear();
        centerYVal.clear();
        angleVal.clear();
        colorID = Eye.NONE_ID;
        //if(DRAW)
        //    image.setTo(new Scalar(0));

        // Detect yellow
        if(favorYellow){
            Core.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH, mask);
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
            if(!contours.isEmpty())
                colorID = Eye.YELLOW_ID;
        }

        // Only process alliance if we didn't want or find any yellow
        if(contours.isEmpty()){
            if(alliance == StandardSetupOpMode.COLOR.RED){
                Core.inRange(hsv, HSV_RED1_LOW, HSV_RED1_HIGH, maskLow);
                Core.inRange(hsv, HSV_RED2_LOW, HSV_RED2_HIGH, maskHi);
                Core.add(maskLow, maskHi, mask);
            }
            else{
                Core.inRange(hsv, HSV_BLUE_LOW, HSV_BLUE_HIGH, mask);
            }
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
            if(!contours.isEmpty())
                colorID = (alliance == StandardSetupOpMode.COLOR.RED) ? Eye.RED_ID : Eye.BLUE_ID;
        }

        // Detect final yellow if no alliance color found and not favor yellow
        if(!favorYellow && contours.isEmpty()){
            Core.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH, mask);
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
            if(!contours.isEmpty())
                colorID = Eye.YELLOW_ID;
        }

        // Process the contours
        for (MatOfPoint contour : contours) {
            // smooth contour
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.005 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approxContour = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxContour, epsilon, true);

            // Draw
            //if (DRAW) {
            //    MatOfPoint mop2f = new MatOfPoint(approxContour.toArray());
            //    List<MatOfPoint> contoursList = new ArrayList<>();
            //    contoursList.add(mop2f);
            //    Imgproc.drawContours(image, contoursList, -1, new Scalar(255), 3);
            //}

            double accumAngle = 0;
            double accumDistance = 0;
            double maxAngle = 0;
            double maxDistance = 0;
            double sumWeights = 0;
            double weightedCentery = 0;
            double weightedCenterx = 0;

            Point[] points = approxContour.toArray();
            for (int i = 1; i < points.length; i++) {
                double midpointX = (points[i - 1].x + points[i].x) / 2;
                double midpointY = (points[i - 1].y + points[i].y) / 2;
                double deltax = points[i].x - points[i - 1].x;
                double deltay = points[i].y - points[i - 1].y;
                if(deltax == 0 || deltay == 0)
                    continue;
                double distance = Math.sqrt(deltay * deltay + deltax * deltax); // Euclidean distance

                // Generate centroid values
                sumWeights += distance;
                weightedCenterx += midpointX * distance;
                weightedCentery += midpointY * distance;

                // Generate line segment angle
                double angle = Math.atan2(deltay, deltax);
                if (angle < 0) {
                    angle += Math.PI;
                }

                // Check angle for longest contiguous straight segment
                if(accumDistance <= 0){
                    accumAngle = angle * distance;
                    accumDistance = distance;
                }
                else{
                    double aveAngle = accumAngle / accumDistance;
                    double diffAngle = aveAngle - angle;
                    if(Math.abs(diffAngle) > Math.PI - MAX_CHANGE)
                        angle += Math.PI * Math.signum(diffAngle);
                    if(Math.abs(aveAngle - angle) > MAX_CHANGE)
                    {
                        if(accumDistance > maxDistance){
                            maxAngle = aveAngle;
                            maxDistance = accumDistance;
                        }
                        accumAngle = angle * distance;
                        accumDistance = distance;
                    }
                    else{
                        accumAngle += angle * distance;
                        accumDistance += distance;
                    }
                }
            }
            weightedCenterx /= sumWeights;
            weightedCentery /= sumWeights;

            // Allow the final segment to wrap into the first segment
            for (int i = 1; i < points.length; i++) {
                double deltax = points[i].x - points[i - 1].x;
                double deltay = points[i].y - points[i - 1].y;
                double distance = Math.sqrt(deltay * deltay + deltax * deltax); // Euclidean distance

                // Generate line segment angle
                double angle = Math.atan2(deltay, deltax);
                if (angle < 0) {
                    angle += Math.PI;
                }

                // Check angle for longest contiguous straight segment
                if(accumDistance <= 0){
                    break;
                }
                else{
                    double aveAngle = accumAngle / accumDistance;
                    double diffAngle = aveAngle - angle;
                    if(Math.abs(diffAngle) > Math.PI - MAX_CHANGE)
                        angle += Math.PI * Math.signum(diffAngle);
                    if(Math.abs(aveAngle - angle) > MAX_CHANGE)
                    {
                        if(accumDistance > maxDistance){
                            maxAngle = aveAngle;
                        }
                        break;
                    }
                    else{
                        accumAngle += angle * distance;
                        accumDistance += distance;
                    }
                }
            }

            // Save values for later
            centerYVal.add((int)weightedCentery);
            centerXVal.add((int)weightedCenterx);
            angleVal.add(maxAngle);
        }

        // Draw
        if (DRAW) {
            Imgproc.drawContours(hsv, contours, -1, new Scalar(0, 0, 255), 1);
            for (int i = 0; i < centerXVal.size(); i++) {
                double centerx = centerXVal.get(i);
                double centery = centerYVal.get(i);
                double angle = angleVal.get(i);
                Imgproc.circle(hsv, new Point(centerx, centery), 3, new Scalar(0, 0, 128), -1);
                double endx = centerx + 50 * Math.cos(angle);
                double endy = centery + 50 * Math.sin(angle);
                Imgproc.line(hsv, new Point(centerx, centery), new Point(endx, endy), new Scalar(0, 0, 128), 10);
            }
            return hsv;
        }
        else
            return input;
    }

    /**
     * Returns the centroid y value of the highest contour.  If no contours then -1 is returned
     * @param contours Contours to search
     * @return The y position of the highest contour centroid (-1 if none found)
     */
    private static int getHighestContourY(List<MatOfPoint> contours){
        int minY = -1;
        for (MatOfPoint contour : contours) {
            Moments moments = Imgproc.moments(contour);
            int y = (int)Math.round(moments.get_m01() / moments.get_m00());
            if (minY < 0 || y < minY) {
                minY = y;
            }
        }
        return minY;
    }

    public Mat matToFloor(Mat input, StandardSetupOpMode.COLOR alliance)
    {
        // Reset (no movement if nothing is found)
        floor_contours.clear();
        floor_left = 0;
        floor_forward = 0;
        cx = 0;
        cy = 0;

        // Convert slices form RGB to HSV
        Rect ROI = new Rect(50, input.height()/2, input.width()-100, input.height()/2);
        Imgproc.cvtColor(input.submat(ROI), hsv_floor, Imgproc.COLOR_RGB2HSV);

        // Mask based on color to detect specimen with clip notch
        if (alliance == StandardSetupOpMode.COLOR.BLUE) {
            Core.inRange(hsv_floor, HSV_BLUE_LOW, HSV_BLUE_HIGH, floor_mask);
        }else{
            Core.inRange(hsv_floor, HSV_RED1_LOW, HSV_RED1_HIGH, floor_low_mask);
            Core.inRange(hsv_floor, HSV_RED2_LOW, HSV_RED2_HIGH, floor_high_mask);
            Core.add(floor_low_mask, floor_high_mask, floor_mask);
        }

        // Find large contours in mask and fill them
        Imgproc.findContours(floor_mask, floor_contours, floorHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        floor_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= 1000);
        Imgproc.drawContours(floor_mask, floor_contours, -1, new Scalar(255), Imgproc.FILLED);

        // Locate specimen clip closest to our target point
        int wd = image.width() * image.width() + image.height() * image.height();
        for (MatOfPoint contour : floor_contours) {
            // Excribe contour for region of interest
            int min_x = Integer.MAX_VALUE;
            int max_x = -Integer.MAX_VALUE;
            int min_y = Integer.MAX_VALUE;
            int max_y = -Integer.MAX_VALUE;
            List<Point> points = contour.toList();
            for (Point point : points) {
                if(point.x < min_x) min_x = (int)point.x;
                if(point.x > max_x) max_x = (int)point.x;
                if(point.y < min_y) min_y = (int)point.y;
                if(point.y > max_y) max_y = (int)point.y;
            }

            // Start at the top and look for the first row with a clip notch
            boolean foundNotch = false;
            for (int y = min_y; y < max_y; y++) {
                // Look for the left-up and left-down notch
                int lu = -1;
                int ld = -1;
                for (int x = min_x; x < max_x; x++) {
                    if (lu < 0 && floor_mask.get(y, x)[0] == 255) {
                        lu = x;
                    } else if (lu >= 0 && ld < 0 && floor_mask.get(y, x)[0] == 0) {
                        ld = x;
                    }
                    if (lu >= 0 && ld >= 0) {
                        break;
                    }
                }

                // Look for the right-up and right-down notch
                int ru = -1;
                int rd = -1;
                for (int x = max_x - 1; x >= min_x; x--) {
                    if (ru < 0 && floor_mask.get(y, x)[0] == 255) {
                        ru = x;
                    } else if (ru >= 0 && rd < 0 && floor_mask.get(y, x)[0] == 0) {
                        rd = x;
                    }
                    if (ru >= 0 && rd >= 0) {
                        break;
                    }
                }

                // Check if the left-down and right-down notches are valid
                if (ld >= 0 && rd >= ld) {
                    foundNotch = true;
                    int tcx = (ld + rd) / 2;
                    int tcy = y;

                    int dx = FLOOR_ALIGNED_X - tcx;
                    int dy = FLOOR_ALIGNED_Y - tcy;
                    int dd = dx * dx + dy * dy;
                    // If closest so far, convert to shift amounts and save position
                    // The motor gets detected at 200 pixels away (more than 200 away ignore)
                    if (dd < 200 * 200 && dd < wd) {
                        cx = tcx;
                        cy = tcy;
                        floor_left = (double)dx * IN_PER_PIXEL_LR;
                        floor_forward = (double)dy * IN_PER_PIXEL_FB;
                        wd = dd;
                    }

                    break;
                }
            }

            // No notch, fall back to center and top of contour
            if(!foundNotch){
                int tcx = (min_x + max_x) / 2;
                int tcy = min_y;

                int dx = FLOOR_ALIGNED_X - tcx;
                int dy = FLOOR_ALIGNED_Y - tcy;
                int dd = dx * dx + dy * dy;
                // If closest so far, convert to shift amounts and save position
                // The motor gets detected at 200 pixels away (more than 200 away ignore)
                if (dd < 200 * 200 && dd < wd) {
                    cx = tcx;
                    cy = tcy;
                    floor_left = (double)dx * IN_PER_PIXEL_LR;
                    floor_forward = (double)dy * IN_PER_PIXEL_FB;
                    wd = dd;
                }
            }
        }

        // Debug
        /*
        telemetry.addData("Num Contours", floor_contours.size());
        telemetry.addData("Floor Left", floor_left);
        telemetry.addData("Floor Forward",floor_forward);
        telemetry.addData("Sum of All Points", Core.sumElems(floor_mask));
        telemetry.update();
        */


        // Just return the original input Mat
        Core.flip(floor_mask, floor_mask, -1);
        return floor_mask;
    }

    public Mat matToBar(Mat input, StandardSetupOpMode.COLOR alliance)
    {
        // Reset
        bar_contours.clear();
        //bar_left_contours.clear();
        //bar_right_contours.clear();
        bar_y = -1;
        //bar_left_y = -1;
        //bar_right_y = -1;

        // Convert slices from RGB to HSV
        Rect centerROI = new Rect( 230-(BAR_SAMPLE_WIDTH/2), 0, BAR_SAMPLE_WIDTH, input.height());
        Imgproc.cvtColor(input.submat(centerROI), bar_slice, Imgproc.COLOR_RGB2HSV);

        //Rect leftROI = new Rect(0, 0, BAR_SAMPLE_WIDTH, input.height());
        //Rect rightROI = new Rect(input.width()-BAR_SAMPLE_WIDTH-1, 0, BAR_SAMPLE_WIDTH, input.height());
        //Imgproc.cvtColor(input.submat(leftROI), bar_left_slice, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(input.submat(rightROI), bar_right_slice, Imgproc.COLOR_RGB2HSV);

        // Detect bar
        if (alliance == StandardSetupOpMode.COLOR.BLUE) {
            Core.inRange(bar_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, bar_mask);
            //Core.inRange(bar_left_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, bar_left_mask);
            //Core.inRange(bar_right_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, bar_right_mask);
        }else{
            Core.inRange(bar_slice, HSV_RED1_LOW, HSV_RED1_HIGH, bar_low_mask);
            Core.inRange(bar_slice, HSV_RED2_LOW, HSV_RED2_HIGH, bar_high_mask);
            Core.add(bar_low_mask, bar_high_mask, bar_mask);
            //Core.inRange(bar_left_slice, HSV_RED1_LOW, HSV_RED1_HIGH, bar_low_mask);
            //Core.inRange(bar_left_slice, HSV_RED2_LOW, HSV_RED2_HIGH, bar_high_mask);
            //Core.add(bar_low_mask, bar_high_mask, bar_left_mask);
            //Core.inRange(bar_right_slice, HSV_RED1_LOW, HSV_RED1_HIGH, bar_low_mask);
            //Core.inRange(bar_right_slice, HSV_RED2_LOW, HSV_RED2_HIGH, bar_high_mask);
            //Core.add(bar_low_mask, bar_high_mask, bar_right_mask);
        }

        // Locate bar end centers (left and right edges)
        Imgproc.findContours(bar_mask, bar_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bar_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= BAR_MIN_AREA);
        //Imgproc.findContours(bar_left_mask, bar_left_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.findContours(bar_right_mask, bar_right_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //bar_left_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= BAR_MIN_AREA);
        //bar_right_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= BAR_MIN_AREA);

        // Find the largest contour (the one with the max area)
        bar_y = getHighestContourY(bar_contours);
        //bar_left_y = getHighestContourY(bar_left_contours);
        //bar_right_y = getHighestContourY(bar_right_contours);

        // Just give them the original frame
        return input;
    }

    public int checkHoldingColor(Mat input) {
        // switch to HSV color space
        Imgproc.cvtColor(input.submat(GRABBED_SAMPLE_ROI), hsv_grabbed, Imgproc.COLOR_RGB2HSV);
        // check amount of yellow
        Core.inRange(hsv_grabbed, HSV_YELLOW_LOW, HSV_YELLOW_HIGH, grabbed_mask);
        int yellowNum = Core.countNonZero(grabbed_mask);
        // check amount of red
        Core.inRange(hsv_grabbed, HSV_RED1_LOW, HSV_RED1_HIGH, grabbed_low_mask);
        Core.inRange(hsv_grabbed, HSV_RED2_LOW, HSV_RED2_HIGH, grabbed_high_mask);
        Core.add(grabbed_low_mask, grabbed_high_mask, grabbed_mask);
        int redNum = Core.countNonZero(grabbed_mask);
        // check amount of blue
        Core.inRange(hsv_grabbed, HSV_BLUE_LOW, HSV_BLUE_HIGH, grabbed_mask);
        Core.countNonZero(grabbed_mask);
        int blueNum = Core.countNonZero(grabbed_mask);
        // set color id
        int MIN_GRABBED_MASK = 50;
        if (redNum > yellowNum && redNum > blueNum && redNum > MIN_GRABBED_MASK) {
            // we grabbed a red
            colorID = Eye.RED_ID;
        } else if (blueNum > yellowNum && blueNum > MIN_GRABBED_MASK) {
            // we grabbed a blue
            colorID = Eye.BLUE_ID;
        } else if (yellowNum > MIN_GRABBED_MASK){
            // we grabbed a yellow
            colorID = Eye.YELLOW_ID;
        } else {
            // we did not grab a color
            colorID = Eye.NONE_ID;
        }

        return colorID;
    }
}
