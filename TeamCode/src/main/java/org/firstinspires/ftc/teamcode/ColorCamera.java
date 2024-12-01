package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

public class ColorCamera extends Thread {

    public final HuskyLens huskyLens;
    private final RevBlinkinLedDriver blinkin;

    // variable to store the color of alliance
    private static final int NONE_ID = 0;
    private static final int YELLOW_ID = 1;
    private static final int RED_ID = 2;
    private static final int BLUE_ID = 3;
    private final int colorId;

    // Point for good pick-up
    public static final int TARGET_X = 160;
    public static final int TARGET_Y = 186;

    public ColorCamera(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color) {
        // Camera setup
        this.huskyLens =  hardwareMap.get(HuskyLens.class, "huskylens");
        colorId = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;

        // LED setup
        this.blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }

    public int getCapturedBlock()
    {
        return NONE_ID;
    }

    /**
     * Gets the block that is closest to the camera and has the same color as we are looking for
     * @return The closest block or null if there are no blocks
     */
    public HuskyLens.Block getClosestBlock() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        // Find yellow blocks first
        HuskyLens.Block[] yellowBlocks = huskyLens.blocks(YELLOW_ID);
        HuskyLens.Block[] redBlocks = huskyLens.blocks(RED_ID);
        HuskyLens.Block[] blueBlocks = huskyLens.blocks(BLUE_ID);
        ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
        blocksOnScreen.addAll(Arrays.asList(yellowBlocks));

        // No yellow blocks, find alliance blocks
        if (blocksOnScreen.isEmpty()) {
            blocksOnScreen.addAll(Arrays.asList(colorId == RED_ID ? redBlocks : blueBlocks));
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
        arrows.addAll(Arrays.asList(huskyLens.arrows()));
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
                //if (dist < smallestDist && arrowCenterX > leftEdgeX && arrowCenterX < rightEdgeX && arrowCenterY < topEdgeY && arrowCenterY > bottomEdgeY) {
                if (dist < smallestDist) {
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

    public double findAngleOfArrow(HuskyLens.Arrow arrow) {
        // find the slope of line
        double deltaX = arrow.y_origin - arrow.y_target;
        double deltaY = arrow.x_origin - arrow.x_target;
        double angle = Math.atan2(deltaY, deltaX);
        // return the arctan of the slope
        return Math.toDegrees(angle);
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

            // If the gamepad is pressed search and pickup

            // Short sleep to keep this loop from saturating
            try {
                sleep(BodyPart.LOOP_PAUSE_MS);
            } catch (InterruptedException e) {
                interrupt();
            }
        }
    }
}
