package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

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
    private final Mat image;

    public List<MatOfPoint> contours = new ArrayList<>();
    public List<Integer> centerXVal = new ArrayList<>();
    public List<Integer> centerYVal = new ArrayList<>();
    public List<Double> angleVal = new ArrayList<>();

    private static final boolean DRAW = true;
    private final int MIN_AREA;

    private final Mat bar_left_slice;
    private final Mat bar_right_slice;
    private final Mat bar_left_mask;
    private final Mat bar_right_mask;
    private final Mat bar_low_mask;
    private final Mat bar_high_mask;

    public List<MatOfPoint> bar_left_contours = new ArrayList<>();
    public List<MatOfPoint> bar_right_contours = new ArrayList<>();
    public int bar_left_y;
    public int bar_right_y;

    private final int BAR_SAMPLE_WIDTH = 20;
    private final int BAR_MIN_AREA = BAR_SAMPLE_WIDTH * 10;

    private static final double MAX_CHANGE = Math.toRadians(10);

    private static final Scalar HSV_YELLOW_LOW = new Scalar(20, 100, 100);
    private static final Scalar HSV_YELLOW_HIGH = new Scalar(40, 255, 255);
    private static final Scalar HSV_RED1_LOW = new Scalar(0, 100, 100);
    private static final Scalar HSV_RED1_HIGH = new Scalar(10, 255, 255);
    private static final Scalar HSV_RED2_LOW = new Scalar(170, 100, 100);
    private static final Scalar HSV_RED2_HIGH = new Scalar(180, 255, 255);
    private static final Scalar HSV_BLUE_LOW = new Scalar(100, 100, 100);
    private static final Scalar HSV_BLUE_HIGH = new Scalar(140, 255, 255);

    public FrameProcessing( int width, int height){
        hsv = new Mat(height, width * 3, CvType.CV_8U);
        mask = new Mat(height, width, CvType.CV_8UC1);
        maskLow = new Mat(height, width, CvType.CV_8UC1);
        maskHi = new Mat(height, width, CvType.CV_8UC1);
        hierarchy = new Mat();
        if(DRAW)
            image = new Mat(height, width, CvType.CV_8U);
        MIN_AREA = (int)Math.round((double)(width * height) * 0.005);

        bar_left_slice = new Mat(height, BAR_SAMPLE_WIDTH * 3, CvType.CV_8U);
        bar_right_slice = new Mat(height, BAR_SAMPLE_WIDTH * 3, CvType.CV_8U);
        bar_left_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        bar_right_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        bar_low_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
        bar_high_mask = new Mat(height, BAR_SAMPLE_WIDTH, CvType.CV_8UC1);
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
        //if(DRAW)
        //    image.setTo(new Scalar(0));

        // Detect yellow
        if(favorYellow){
            Core.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH, mask);
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
        }

        // Only process alliance if we didn't want or find any yellow
        if(contours.size() == 0){
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
     * Returns the centroid y value of the largest contour.  If no contours then -1 is returned
     * @param contours Contours to search
     * @return The y position of the largest contour centroid (-1 if none found)
     */
    private static int getLargestContourY(List<MatOfPoint> contours){
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        if(largestContour != null){
            Moments moments = Imgproc.moments(largestContour);
            return (int)Math.round(moments.get_m01() / moments.get_m00());
        }
        return -1;
    }

    public Mat matToBar(Mat input, StandardSetupOpMode.COLOR alliance)
    {
        // Reset
        bar_left_contours.clear();
        bar_right_contours.clear();
        bar_left_y = -1;
        bar_right_y = -1;

        // Convert slices from RGB to HSV
        Rect leftROI = new Rect(0, 0, BAR_SAMPLE_WIDTH, input.height());
        Rect rightROI = new Rect(input.width()-BAR_SAMPLE_WIDTH-1, 0, BAR_SAMPLE_WIDTH, input.height());
        Imgproc.cvtColor(input.submat(leftROI), bar_left_slice, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(input.submat(rightROI), bar_right_slice, Imgproc.COLOR_BGR2HSV);

        // Detect bar
        if (alliance == StandardSetupOpMode.COLOR.BLUE) {
            Core.inRange(bar_left_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, bar_left_mask);
            Core.inRange(bar_right_slice, HSV_BLUE_LOW, HSV_BLUE_HIGH, bar_right_mask);
        }else{
            Core.inRange(bar_left_slice, HSV_RED1_LOW, HSV_RED1_HIGH, bar_low_mask);
            Core.inRange(bar_left_slice, HSV_RED2_LOW, HSV_RED2_HIGH, bar_high_mask);
            Core.add(bar_low_mask, bar_high_mask, bar_left_mask);
            Core.inRange(bar_right_slice, HSV_RED1_LOW, HSV_RED1_HIGH, bar_low_mask);
            Core.inRange(bar_right_slice, HSV_RED2_LOW, HSV_RED2_HIGH, bar_high_mask);
            Core.add(bar_low_mask, bar_high_mask, bar_right_mask);
        }

        // Locate bar end centers (left and right edges)
        Imgproc.findContours(bar_left_mask, bar_left_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(bar_right_mask, bar_right_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bar_left_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= BAR_MIN_AREA);
        bar_right_contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= BAR_MIN_AREA);

        // Find the largest contour (the one with the max area)
        bar_left_y = getLargestContourY(bar_left_contours);
        bar_right_y = getLargestContourY(bar_right_contours);

        // Just give them the original frame
        return input;    }
}