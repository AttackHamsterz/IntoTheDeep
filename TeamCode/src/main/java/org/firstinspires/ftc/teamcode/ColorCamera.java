package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.EnumMap;

    @TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
    public class ColorCamera extends LinearOpMode {

        private HuskyLens huskyLens;
        // variable to store the color id we don't want to look for
        private int excludeColorId;
        // the largest height we'd expect from a valid game piece
        private int maxHeight;
        // the largest width we'd expect from a valid game piece
        private int maxWidth;
        // center of the screen
        private final int CENTER_X = 160;
        private final int CENTER_Y = 120;
        // a list to store the blocks with the same item we are looking for
        private ArrayList<HuskyLens.Block> blocksOfColor = new ArrayList<>();

        public ColorCamera(String color) {
            // sets the color that we are looking for
            if (color.equalsIgnoreCase("red")) {
                excludeColorId = 2;
            } else if (color.equalsIgnoreCase("blue")) {
                excludeColorId = 3;
            } else {
                throw new IllegalArgumentException("Please input a valid color");
            }
        }



        /**
         * Gets the block that is closest to the camera and has the same color as we are looking for
         * @return The closest block or null if there are no blocks
         */
        public HuskyLens.Block getClosestBlock() {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
            for (int i = 0; i < blocks.length; i++) {
                // add any yellow blocks
                if (blocks[i].id == 1) {
                    blocksOnScreen.add(blocks[i]);
                }
            }
            if (blocksOnScreen.isEmpty()) {
                for (int i = 0; i < blocks.length; i++) {
                    // look for any blocks that are not the color we are excluding
                    if (blocks[i].id != excludeColorId) {
                        blocksOnScreen.add(blocks[i]);
                    }
                }
            }
            // make sure there are blocks on the screen with the color we are looking for
            if (!blocksOfColor.isEmpty()) {
                // 400 is the longest distance possible on the camera screen
                double smallestDist = 400;
                int smallestDistIndex = 0;
                // use pythagorean theorem to draw a line from the camera to each block's position
                // whichever block has the shortest distance is the block we are closest to
                for (int i = 0; i < blocksOfColor.size(); i++) {
                    // get the distance of the block from the center of the screen
                    int xDiff = Math.abs(CENTER_X - blocksOfColor.get(i).x);
                    int yDiff = Math.abs(CENTER_Y - blocksOfColor.get(i).y);

                    double dist = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

                    if (dist < smallestDist) {
                        smallestDist = dist;
                        smallestDistIndex = i;
                    }
                }

                return blocksOfColor.get(smallestDistIndex);
            }
            return null;
        }

        @Override
        public void runOpMode() {
            // get the camera from the config
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

            // check to see if the device is working
            if (!huskyLens.knock()) {
                telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
            } else {
                telemetry.addData(">>", "Press start to continue");
            }

            // start the color recognition algorithm
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

            telemetry.update();
            waitForStart();

            while(opModeIsActive()) {

                // the color tracking algorithm returns a block around the location of the colored object
                // from that block, we can figure out the location of our object
                HuskyLens.Block[] blocks = huskyLens.blocks();
                // tell us how many block we can see
                telemetry.addData("Block count", blocks.length);
                // go through every block we can see and print out information about that block
                //ArrayList<HuskyLens.Block> blocksOnScreen = new ArrayList<>();
                for (HuskyLens.Block block : blocks) {
                    telemetry.addData("Block", block.toString());
                    // checks the color id of what we are looking at to see if it is the one we are looking for
                    /*
                    if (blocks[i].id != excludeColorId) {
                        // check if
                        // add the block to a list
                        blocksOnScreen.add(blocks[i]);
                    }

                     */
                }
               // blocksOfColor = blocksOnScreen;
                telemetry.update();

            }
        }

    }
