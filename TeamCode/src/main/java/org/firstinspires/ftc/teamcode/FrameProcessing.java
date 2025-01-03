package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * A class to perform frame processing in opencv.  Also lets us easily unit test the algorithm
 */
public class FrameProcessing {
    private final Mat lab;
    private final Mat mask;
    private final Mat heirarchy;

    public List<MatOfPoint> contours = new ArrayList<>();
    public List<Integer> centerXVal = new ArrayList<>();
    public List<Integer> centerYVal = new ArrayList<>();
    public List<Double> angleVal = new ArrayList<>();

    private final int MIN_AREA;

    private static final double MAX_CHANGE = Math.toRadians(10);
    private static final Scalar YELLOW_LOW = new Scalar(220, 110, 150);
    private static final Scalar YELLOW_HIGH = new Scalar(255, 125, 220);
    private static final Scalar RED_LOW = new Scalar(160, 140, 125);
    private static final Scalar RED_HIGH = new Scalar(240, 190, 190);
    private static final Scalar BLUE_LOW = new Scalar(70, 140, 45);
    private static final Scalar BLUE_HIGH = new Scalar(150, 170, 65);

    public FrameProcessing( int width, int height){
        lab = new Mat(height, width * 3, CvType.CV_8U);
        mask = new Mat(height, width, CvType.CV_8UC1);
        heirarchy = new Mat();

        MIN_AREA = (int)Math.round((double)(width * height) * 0.005);
    }

    public void matToDetection(Mat input, StandardSetupOpMode.COLOR alliance, boolean favorYellow)
    {
        // Convert from BGR to LAB
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_BGR2Lab);

        // Clear existing detections
        contours.clear();
        centerXVal.clear();
        centerYVal.clear();
        angleVal.clear();

        // Detect yellow
        if(favorYellow){
            Core.inRange(lab, YELLOW_LOW, YELLOW_HIGH, mask);
            Imgproc.findContours(mask, contours, heirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
        }

        // Only process alliance if we didn't want or find any yellow
        if(contours.size() == 0){
            if(alliance == StandardSetupOpMode.COLOR.RED){
                Core.inRange(lab, RED_LOW, RED_HIGH, mask);
            }
            else{
                Core.inRange(lab, BLUE_LOW, BLUE_HIGH, mask);
            }
            Imgproc.findContours(mask, contours, heirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            contours.removeIf(cnt -> Imgproc.contourArea(cnt) <= MIN_AREA);
        }

        // Process the contours
        for (MatOfPoint contour : contours) {
            // smooth contour
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.005 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approxContour = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxContour, epsilon, true);

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
    }

    public void matToBar(Mat input)
    {

    }
}
