package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

public class ColorCamera extends Thread {

    public HuskyLens huskyLens;
    private RevBlinkinLedDriver blinkin;

    // variable to store the color of alliance
    private static int YELLOW_ID = 1;
    private static int RED_ID = 2;
    private static int BLUE_ID = 3;
    private StandardSetupOpMode.COLOR allianceColor;
    private int colorId;

    // the largest height we'd expect from a valid game piece
    private int maxHeight;
    // the largest width we'd expect from a valid game piece
    private int maxWidth;
    // Point for good pick-up
    public static final int TARGET_X = 160;
    public static final int TARGET_Y = 186;

    // Search variables
    public static final int CLOSE_ENOUGH = 5;


    public ColorCamera(HardwareMap hardwareMap, StandardSetupOpMode.COLOR color) {
        // Camera setup
        this.huskyLens =  hardwareMap.get(HuskyLens.class, "huskylens");
        this.allianceColor = color;
        colorId = (color == StandardSetupOpMode.COLOR.RED) ? RED_ID : BLUE_ID;

        // LED setup
        this.blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
    }

    /**
     * Gets the block that is closest to the camera and has the same color as we are looking for
     * @return The closest block or null if there are no blocks
     */
    public HuskyLens.Block getClosestBlock() {
        // Find yellow blocks first
        HuskyLens.Block[] yellowBlocks = huskyLens.blocks(YELLOW_ID);
        HuskyLens.Block[] redBlocks = huskyLens.blocks(RED_ID);
        HuskyLens.Block[] blueBlocks = huskyLens.blocks(BLUE_ID);
        ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
        blocksOnScreen.addAll(Arrays.asList(yellowBlocks));

        // No yellow blocks, find alliance blocks
        if (blocksOnScreen.isEmpty()) {
            HuskyLens.Block[] allianceBlocks = huskyLens.blocks(colorId);
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
                if (dist < smallestDist) {
                    smallestDist = dist;
                    smallestDistIndex = i;
                }
            }

            return blocksOnScreen.get(smallestDistIndex);
        }
        return null;
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
