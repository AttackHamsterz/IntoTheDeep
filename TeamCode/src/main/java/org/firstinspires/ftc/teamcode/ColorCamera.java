package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ColorCamera extends Thread {

    public final HuskyLens huskyLens;
    private final RevBlinkinLedDriver blinkin;

    // variable to store the color of alliance
    private static final int NONE_ID = 0;
    private static final int YELLOW_ID = 1;
    private static final int RED_ID = 2;
    private static final int BLUE_ID = 3;
    private final int colorId;

    // Capture area - TODO get real values
    public static final int CAPTURE_X1 = 180 - 60;
    public static final int CAPTURE_Y1 = 58 - 60;
    public static final int CAPTURE_X2 = 180 + 60;
    public static final int CAPTURE_Y2 = 58 + 60;

    // Point for good pick-up
    public static final int TARGET_X = 160;
    public static final int TARGET_Y = 186;

    // Other variables
    private static final int MIN_DETECTION_EDGE_SIZE = 5;

    // Arm calibration values (ensure shoulder is in
    private static final int NEAR_Y = 160;
    private static final int FAR_Y = 50;
    private static final int NEAR_TICKS = 0;
    private static final int FAR_TICKS = 550;

    // y=mx+b where y is ticks and x is the relative y pixel location
    private static final double M = (double)(FAR_TICKS - NEAR_TICKS) / (double)(FAR_Y - NEAR_Y);
    private static final double B = (double) FAR_TICKS - (M * (double) FAR_Y);

    // Leg calibration values
    private static final int CENTER_X = 160;
    private static final int INCHES_FROM_CENTER = 4;
    private static final int SHIFT_NEAR_X = 283;
    private static final int SHIFT_NEAR_Y = 139;
    private static final double SHIFT_NEAR_M = (double)INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_NEAR_X);

    private static final int SHIFT_FAR_X = 252;
    private static final int SHIFT_FAR_Y = 109;
    private static final double SHIFT_FAR_M = (double) INCHES_FROM_CENTER / (double)(CENTER_X - SHIFT_FAR_X) ;

    // how much our slop is changing based on y
    private static final double SHIFT_M = (SHIFT_FAR_M - SHIFT_NEAR_M) / (double) (SHIFT_FAR_Y - SHIFT_NEAR_Y);
    private static final double SHIFT_B = SHIFT_NEAR_M - (SHIFT_M * (double) SHIFT_NEAR_Y);

    protected MecanumDrive legs;
    protected Arm arm;
    protected Shoulder shoulder;
    protected Hand hand;
    protected Gamepad gamepad;

    protected boolean ignoreGamepad = false;
    protected boolean favorYellow = false;
    protected final Pose2d startPose = new Pose2d(0,0,0);

    public ColorCamera(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color, MecanumDrive legs, Arm arm, Shoulder shoulder, Hand hand, Gamepad gamepad, boolean favorYellow) {
        // Camera setup
        this.huskyLens =  hardwareMap.get(HuskyLens.class, "huskylens");
        colorId = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;

        this.legs = legs;
        this.arm = arm;
        this.shoulder = shoulder;
        this.hand = hand;
        this.gamepad = gamepad;
        this.favorYellow = favorYellow;

        // LED setup
        this.blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        if(colorId == BLUE_ID)
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        else
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
    }

    /**
     * Get the id of the first block completely contained in the capture area
     *
     * @return captured id
     */
    public int getCapturedBlockId()
    {
        // Find color blocks
        HuskyLens.Block[] yellowBlocks = huskyLens.blocks(YELLOW_ID);
        HuskyLens.Block[] redBlocks = huskyLens.blocks(RED_ID);
        HuskyLens.Block[] blueBlocks = huskyLens.blocks(BLUE_ID);

        ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
        blocksOnScreen.addAll(Arrays.asList(yellowBlocks));
        blocksOnScreen.addAll(Arrays.asList(redBlocks));
        blocksOnScreen.addAll(Arrays.asList(blueBlocks));
        blocksOnScreen.removeIf(b -> b.x - b.width / 2 < CAPTURE_X1 || b.x + b.width / 2 > CAPTURE_X2 || b.y - b.height / 2 < CAPTURE_Y1 || b.y + b.height / 2 > CAPTURE_Y2);

        return blocksOnScreen.isEmpty() ? NONE_ID : blocksOnScreen.get(0).id;
    }

    /**
     * If we have our alliance color or yellow captured
     * @return true if captured
     */
    public boolean blockCaptured()
    {
        int capturedId = getCapturedBlockId();
        return capturedId == colorId || capturedId == YELLOW_ID;
    }

    /**
     * Method that toggles if we are ignoring gamepad input (true for autonomous)
     * @param ignoreGamepad
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }

    /**
     * Gets the block that is closest to the camera and has the same color as we are looking for
     * @return The closest block or null if there are no blocks
     */
    public HuskyLens.Block getClosestBlock() {

        // Find color blocks
        HuskyLens.Block[] yellowBlocks = huskyLens.blocks(YELLOW_ID);
        HuskyLens.Block[] redBlocks = huskyLens.blocks(RED_ID);
        HuskyLens.Block[] blueBlocks = huskyLens.blocks(BLUE_ID);

        // Keep track of the blocks on screen
        ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();

        // add all blocks of our alliance
        if(favorYellow)
            blocksOnScreen.addAll(Arrays.asList(yellowBlocks));
        else
            blocksOnScreen.addAll(Arrays.asList(colorId == RED_ID ? redBlocks : blueBlocks));

        // Remove tiny blocks that are false alarms (detections must be MIN_DETECTION_EDGE_SIZE)
        blocksOnScreen.removeIf(b -> b.width < MIN_DETECTION_EDGE_SIZE || b.height < MIN_DETECTION_EDGE_SIZE);

        // No alliance blocks, find yellow blocks that are large enough
        if (blocksOnScreen.isEmpty()) {
            if(favorYellow)
                blocksOnScreen.addAll(Arrays.asList(colorId == RED_ID ? redBlocks : blueBlocks));
            else
                blocksOnScreen.addAll(Arrays.asList(yellowBlocks));

            // Filter small
            blocksOnScreen.removeIf(b -> b.width < MIN_DETECTION_EDGE_SIZE || b.height < MIN_DETECTION_EDGE_SIZE);
        }

        // make sure there are blocks on the screen with the color we are looking for
        if (!blocksOnScreen.isEmpty()) {
            // Find closest block to our target
            double smallestDist = 1000;
            int smallestDistIndex = 0;
            // use pythagorean theorem to draw a line from the camera to each block's position
            // whichever block has the shortest distance is the block we are closest to
            for (int i = 0; i < blocksOnScreen.size(); i++) {
                // get the distance of the block from the center of the screen
                int xDiff = Math.abs(TARGET_X - blocksOnScreen.get(i).x);
                int yDiff = Math.abs(TARGET_Y - blocksOnScreen.get(i).y);

                double dist = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
                if (dist < smallestDist ) {
                    smallestDist = dist;
                    smallestDistIndex = i;
                }
            }

            return blocksOnScreen.get(smallestDistIndex);
        }
        return null;
    }

    public List<HuskyLens.Arrow> getArrowsInBlock(HuskyLens.Block block){
        int leftEdgeX = block.x - (block.width/2);
        int bottomEdgeY = block.y - (block.width/2);
        int rightEdgeX = block.x + (block.width/2);
        int topEdgeY = block.y + (block.height/2);

        /*
        Point[] polygon = new Point[] {
                new Point(leftEdgeX, bottomEdgeY),
                new Point(leftEdgeX, topEdgeY),
                new Point(rightEdgeX, bottomEdgeY),
                new Point(rightEdgeX, topEdgeY)
        };

         */

        ArrayList<HuskyLens.Arrow> arrows = new ArrayList<>();
        for(HuskyLens.Arrow arrow : huskyLens.arrows()) {
            /*
            LineSegment line = new LineSegment(new Point(arrow.x_origin, arrow.y_origin), new Point(arrow.x_target, arrow.y_target));
            if (doesIntersect(polygon, line)) {
                arrows.add(arrow);
            }
             */

            int arrowCenterX = (arrow.x_origin + arrow.x_target) / 2;
            int arrowCenterY = (arrow.y_origin + arrow.y_target) / 2;
            if(arrowCenterX >= leftEdgeX && arrowCenterX <= rightEdgeX && arrowCenterY <= topEdgeY && arrowCenterY >= bottomEdgeY)
                arrows.add(arrow);


        }
        return arrows;
    }

    /*
    // A helper class to represent a point with x and y coordinates
     class Point {
        double x, y;
        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    // A helper class to represent a line segment between two points
     class LineSegment {
        Point start, end;
        LineSegment(Point start, Point end) {
            this.start = start;
            this.end = end;
        }
    }

    // Utility method to calculate the orientation of the triplet (p, q, r)
    // Returns 0 if collinear, 1 if clockwise, -1 if counterclockwise
    public int orientation(Point p, Point q, Point r) {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : -1; // clockwise or counterclockwise
    }

    // Check if point q lies on the line segment pr
    public boolean onSegment(Point p, Point q, Point r) {
        if (q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
                q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y)) {
            return true;
        }
        return false;
    }

    // Check if two line segments p1q1 and p2q2 intersect
    public boolean doIntersect(LineSegment l1, LineSegment l2) {
        Point p1 = l1.start, q1 = l1.end;
        Point p2 = l2.start, q2 = l2.end;

        // Find the four orientations needed for the general and special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) {
            return true;
        }

        // Special cases
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't intersect
    }

    // Check if a line intersects the polygon (represented by a list of points)
    public boolean doesIntersect(Point[] polygon, LineSegment line) {
        int n = polygon.length;
        for (int i = 0; i < n; i++) {
            // Create edges of the polygon
            Point p1 = polygon[i];
            Point p2 = polygon[(i + 1) % n];
            LineSegment edge = new LineSegment(p1, p2);
            if (doIntersect(edge, line)) {
                return true; // Found an intersection
            }
        }
        return false; // No intersection with any edge
    }

     */


    public HuskyLens.Arrow getClosestArrowToBlock(HuskyLens.Block block) {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
        // get the center of the block
        int centerX = block.x;
        int centerY = block.y;

        int leftEdgeX = block.x - (block.width/2);
        int bottomEdgeY = block.y - (block.width/2);
        int rightEdgeX = block.x + (block.width/2);
        int topEdgeY = block.y + (block.height/2);

        // get a list of all arrows on the screen
        ArrayList<HuskyLens.Arrow> arrows = new ArrayList<>();
        arrows.addAll(Arrays.asList(huskyLens.arrows(block.id)));
        // make sure there are arrows on screen
        if (!arrows.isEmpty()) {
            // go through our list and figure out which arrow is closest to the block's location
            double smallestDist = 1000;
            int smallestDistIndex = -1;
            for (int i = 0; i < arrows.size(); i++) {
                // get the center of the arrow
                int arrowCenterX = (arrows.get(i).x_origin + arrows.get(i).x_target) / 2;
                int arrowCenterY = (arrows.get(i).y_origin + arrows.get(i).y_target) / 2;
                // find the distance from the block's center
                int xDiff = Math.abs(centerX - arrowCenterX);
                int yDiff = Math.abs(centerY - arrowCenterY);
                double dist = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
                // check if we are closer to the center of the block
                if (dist < smallestDist && arrowCenterX > leftEdgeX && arrowCenterX < rightEdgeX && arrowCenterY < topEdgeY && arrowCenterY > bottomEdgeY) {
                    smallestDist = dist;
                    smallestDistIndex = i;
                }
            }
            if (smallestDistIndex != -1) {
                return arrows.get(smallestDistIndex);
            } else {
                return null;
            }

        }
        return null;
    }

    /**
     * Method assumes that positive angles are counter clockwise (left)
     * Assumes 0 angle is straight up (no rotation necessary)
     * Assumes 90 angle is straight left
     * Assumes -90 angle is straight right
     * @param arrow arrow to measure angle of
     * @return angle of line in degrees [-90, 90]
     */
    public double findAngleOfArrow(HuskyLens.Arrow arrow) {
        // Find the angle of the line (atan2 is -pi to pi where
        // 0 is the positive x axis, pi/2 is the positive y axis
        // -pi/2 is the negative y axis
        double deltaX = arrow.y_origin - arrow.y_target;
        double deltaY = arrow.x_origin - arrow.x_target;
        double angle = Math.atan2(deltaY, deltaX);

        // If the angle was negative we can just make it positive (same line)
        //if(angle < 0) angle += Math.PI;

        // Rotate the atan2 reference frame to the tool reference frame
        //angle -= Math.PI / 2.0;

        // Return line angle in degrees
        return Math.toDegrees(angle);
    }

    public void autoGrab()
    {
        // Grab the nearest block
        final HuskyLens.Block firstBlock = this.getClosestBlock();
        if (firstBlock != null) {
            //telemetry.addData("block1 id", firstBlock.id);
            // Move the arm and robot to get that block closer
            Action moveArmToBlock = telemetryPacket -> {
                shoulder.setMode(Shoulder.Mode.SEARCH);
                int ticks = (int) Math.round((M * (double) firstBlock.y + B));
                arm.setPosition(1.0, arm.getCurrentPosition() + ticks);
                //telemetry.addData("block1 ticks", ticks);
                return false;
            };
            Action strafeToBlock = telemetryPacket -> {
                double ySlope = firstBlock.y * SHIFT_M + SHIFT_B;
                double shift = (firstBlock.x - CENTER_X) * ySlope;
                //telemetry.addData("block1 shift", shift);
                legs.moveLeft(shift, false);
                return false;
            };
            ParallelAction centerBlockAction = new ParallelAction(
                    new CompleteAction(moveArmToBlock, arm),
                    new CompleteAction(strafeToBlock, legs));
            Actions.runBlocking(centerBlockAction);

            // Get the block again now that it's closer
            final HuskyLens.Block secondBlock = this.getClosestBlock();
            if(secondBlock != null) {
                double averageAngle = 0;

                // Get a rough angle from the block itself
                //averageAngle = Math.toDegrees(Math.atan2(secondBlock.height, secondBlock.width));
                //telemetry.addData("Block Angle", averageAngle);

                // Average a few arrows that fall inside our block
                int numAverage = 0;
                long average_ms = 500;
                long startTime_ms = System.currentTimeMillis();
                this.huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
                do {
                    List<HuskyLens.Arrow> arrows = this.getArrowsInBlock(secondBlock);
                    for(HuskyLens.Arrow arrow : arrows){
                        // Get current angle from this arrow
                        double arrowAngle = this.findAngleOfArrow(arrow);

                        // If this angle has wrapped then we put it near the average
                        // This avoids adding 89.9 + -89.9 to get an average of 0
                        // instead of either 90 or -90
                        if (numAverage > 0) {
                            double currentAverage = averageAngle / (double) numAverage;
                            double deltaAngle = arrowAngle - currentAverage;
                            if (deltaAngle > 160.0)
                                arrowAngle -= 180.0;
                            else if (deltaAngle < -160.0)
                                arrowAngle += 180.0;
                        }

                        // Increment average
                        averageAngle += arrowAngle;
                        numAverage++;
                    }
                    if (numAverage > 6)
                        break;
                } while (System.currentTimeMillis() - startTime_ms < average_ms);
                this.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

                // Set new arm position!
                int ticks = (int) Math.round((M * (double) secondBlock.y + B));
                //telemetry.addData("block2 ticks", ticks);
                arm.setPosition(1.0, arm.getCurrentPosition() + ticks);

                // Set new legs position
                double ySlope = secondBlock.y * SHIFT_M + SHIFT_B;
                double shift = (secondBlock.x - CENTER_X) * ySlope;
                //telemetry.addData("block2 shift", ticks);
                legs.moveLeft(shift, false);

                // Move wrist with a good average
                if (numAverage > 0) {
                    averageAngle /= (double) numAverage;
                    averageAngle = Range.clip(averageAngle, -90.0, 90.0);
                    double wristPos = 0.5 - (0.5*averageAngle / 90.0);
                    //telemetry.addData("Average Angle", averageAngle);
                    //telemetry.addData("Num in average", numAverage);
                    //telemetry.addData("Wrist Pos", wristPos);
                    hand.setWrist(wristPos);

                    // Plunge to pickup

                } //else {
                //telemetry.addLine("No line average");
                //}
            }
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
            //else
            //  telemetry.addLine("block2 null");

            // This is either openCV on image data
            // Or switching to line detection mode
            // And getting arrows if that's quick enough
            // If it's too slow we'll need to go back to
            // USB camera until we get a limelight 3a
        }
        //else
        //telemetry.addLine("block1 null");
        //telemetry.update();

    }

    @Override
    public void run() {
        // check to see if the device is working
        if (!huskyLens.knock())
            return;

        // start the color recognition algorithm
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while (!isInterrupted()) {

            // Find the closest block
            HuskyLens.Block block = getClosestBlock();

            // TODO - is it captured?

            // Indicate color we have locked on to
            if(block == null)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            else if(block.id == YELLOW_ID)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            else if(block.id == RED_ID)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            else if(block.id == BLUE_ID)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            else
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

            // If search is pressed and the shoulder is close enough to search height
            if(!ignoreGamepad && gamepad.x && shoulder.modeReady(Shoulder.Mode.SEARCH)) {
                autoGrab();
            }

            // Short sleep to keep this loop from saturating
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }

        }
    }
}
