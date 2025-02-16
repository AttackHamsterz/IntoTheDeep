package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousLeftFast extends AutonomousOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // If we grabbed a sample from the center, drive and place in lower bucket
        Pose2d lowBucketDropPose = new Pose2d(new Vector2d(20.0 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(162));
        Pose2d highBucketDropPose = new Pose2d(new Vector2d(15 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(162));
        Pose2d colorCheckPose = new Pose2d(new Vector2d(27.0 + X_OFFSET, Y_OFFSET), Math.toRadians(0));
        int lowBucketDropArmPosition = 1730;
        int highBucketDropArmPosition = 2200;

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, highBucketDropArmPosition);
            hand.bucket();
            return false;
        };

        Action releaseSample = telemetryPacket -> {

            hand.halfRelease(RELEASE_MS, false);
            return false;
        };

        Action handReset = telemetryPacket -> {
            hand.hangSample();
            return false;
        };

        Action checkAndSpin = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, colorCheckPose, 1, true), legs),
                handReset
        );
        Actions.runBlocking(checkAndSpin);



        // Check if we have a block
        /*
        boolean haveBlock = eye.getGrabColor() != Eye.NONE_ID;

        if(haveBlock) {
            Action driveToLowBucketDrop = legs.moveToAction(AUTO_POWER, lowBucketDropPose);

            Action raiseShoulderAction = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.LOW_BUCKET, AUTO_POWER, bucketDropArmPosition);
                return false;
            };
            Action driveAndExtendAction = new ParallelAction(
                    new CompleteAction(driveToLowBucketDrop, legs),
                    new CompleteAction(raiseShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm)
            );

            Action binDrop = new SequentialAction(
                    driveAndExtendAction,
                    new CompleteAction(releaseSample, hand));
            Actions.runBlocking(binDrop);
        }

         */

        Action raiseAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BUCKET);
            shoulder.setPositionForMode(Shoulder.Mode.HIGH_BUCKET, AUTO_POWER, highBucketDropArmPosition);
            return false;
        };

        // Repeat this 3 times for each floor sample
        for(int i = 0; i < 3; i++)
        {
            Double wristAngle = 0.35 + (double)i * 0.15;
            //Integer searchPosition = (i==2) ? 1160 : (i==1) ? 920 : 990;
            Integer searchPosition = (i==2) ? 1660 : (i==1) ? 1420 : 1550;


            Action retractForPickupAction = telemetryPacket -> {
                arm.setPosition(0.9, searchPosition);
                return false;
            };

            // Cycle from bucket to floor samples
            Action lowerAction = telemetryPacket -> {
                shoulder.setMode(Shoulder.Mode.SEARCH);
                shoulder.setPositionForMode(Shoulder.Mode.SEARCH, 0.8, searchPosition);
                return false;
            };

            Action wristAction = telemetryPacket -> {
                hand.setWrist(wristAngle);
                return false;
            };

            Action searchAction = telemetryPacket -> {
                hand.setWrist(wristAngle);
                return false;
            };

            // Turn to ground samples, pick one up
            double turnAngle = 1 + ((i==2) ? 32 : (i==1) ? 2 : -29);
            Pose2d pickupPose = new Pose2d(new Vector2d(15 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(turnAngle));
            //int direction = (i==0 && !haveBlock) ? -1 : 1;
            int direction = (i==0) ? -1 : 1;
            Action turnToPickup = legs.moveToAction(0.5, pickupPose, direction);

            Action lower = new ParallelAction(
                    new CompleteAction(retractForPickupAction, arm),
                    new CompleteAction(lowerAction, shoulder)
            );
            Action pauseAndLower = new SequentialAction(
                    new SleepAction(0.35),
                    lower
            );
            Action turnAndLower = new ParallelAction(
                    wristAction,
                    new CompleteAction(turnToPickup, legs),
                    pauseAndLower
            );





            //if (i==0) {
                /*
                Action pickupAction = new SequentialAction(
                        turnAndLower
                );

                Actions.runBlocking(pickupAction);

                Action safeSearch = eye.safeSearch();
                shoulder.setMode(Shoulder.Mode.NONE);
                Actions.runBlocking(safeSearch);
                shoulder.setMode(Shoulder.Mode.NONE);

                 */
            //} else {

                Action grabAction = telemetryPacket -> {
                    shoulder.setMode(Shoulder.Mode.GROUND);
                    shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.6, searchPosition);
                    hand.grab(GRAB_MS);
                    return false;
                };

                Action pickupAction = new SequentialAction(
                        turnAndLower,
                        new CompleteAction(grabAction, hand));
                Actions.runBlocking(pickupAction);
            //}

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(0.4, highBucketDropPose, -1);
            Action turnRaiseAndExtend = new ParallelAction(
                    new CompleteAction(turnToBucket, legs),
                    new CompleteAction(raiseAction, shoulder),
                    new CompleteAction(extendArmAction, arm));
            Action dropAction = new SequentialAction(
                    turnRaiseAndExtend,
                    new CompleteAction(releaseSample, hand)
            );
            Actions.runBlocking(dropAction);
        }


        // We are facing the buckets and we just dropped a sample
        Action resetShoulder = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(0.7, Shoulder.MAX_POS-50);
            return false;
        };
        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };

        //Pose2d parkPose = new Pose2d(new Vector2d(21 + X_OFFSET, 45 + Y_OFFSET), Math.toRadians(-45));
        Pose2d parkPose1 = new Pose2d(new Vector2d(32.5+X_OFFSET, 30+Y_OFFSET), Math.toRadians(90));
        Pose2d parkPose2 = new Pose2d(new Vector2d(56+X_OFFSET, 32+Y_OFFSET), Math.toRadians(90));
        Pose2d parkPose3 = new Pose2d(new Vector2d(56+X_OFFSET, 12.5+Y_OFFSET), Math.toRadians(90));

        Action resetAction = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, parkPose1, 1, true), legs),
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(resetArm, arm));
        Action moveToPark = new SequentialAction(
                resetAction,
                new CompleteAction(legs.moveToAction(AUTO_POWER, parkPose2, 1, true), legs),
                new CompleteAction(legs.moveToAction(0.5, parkPose3, 1, true), legs)
        );
        Actions.runBlocking(moveToPark);

        sleep(1500);
    }
}