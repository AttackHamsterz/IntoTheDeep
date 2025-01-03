package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousLeft extends AutonomousOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // If we grabbed a sample from the center, drive and place in lower bucket
        Pose2d lowBucketDropPose = new Pose2d(new Vector2d(20.0 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(164));
        Pose2d colorCheckPose = new Pose2d(new Vector2d(27.0 + X_OFFSET, Y_OFFSET), Math.toRadians(90));
        int bucketDropArmPosition = 1730;

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, bucketDropArmPosition);
            hand.bucket();
            return false;
        };

        Action releaseSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };

        Actions.runBlocking(new CompleteAction(legs.moveToAction(AUTO_POWER, colorCheckPose, 1, true), legs));

        boolean haveBlock = camera.blockCaptured();

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

        Action raiseAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.LOW_BUCKET, AUTO_POWER, bucketDropArmPosition);
            return false;
        };

        // Repeat this 3 times for each floor sample
        for(int i = 0; i < 3; i++)
        {
            Double wristAngle = 0.35 + (double)i * 0.15;
            Integer searchPosition = (i==2) ? 1160 : (i==1) ? 920 : 990;

            Action retractForPickupAction = telemetryPacket -> {
                arm.setPosition(0.9, searchPosition);
                return false;
            };

            // Cycle from bucket to floor samples
            Action lowerAction = telemetryPacket -> {
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
            Pose2d pickupPose = new Pose2d(new Vector2d(20.87 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(turnAngle));
            int direction = (i==0 && !haveBlock) ? -1 : 1;
            Action turnToPickup = legs.moveToAction(AUTO_POWER, pickupPose, direction);

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
            Action grabAction = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.6, searchPosition);
                hand.grab(GRAB_MS);
                return false;
            };
            Action pickupAction = new SequentialAction(
                    turnAndLower,
                    new CompleteAction(grabAction, hand));
            Actions.runBlocking(pickupAction);

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(0.7, lowBucketDropPose, -1);
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

        // TODO - Park in climb 1 area touching the bar
        // We are facing the buckets and we just dropped a sample
        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };

        Pose2d parkPose = new Pose2d(new Vector2d(21 + X_OFFSET, 45 + Y_OFFSET), Math.toRadians(-45));
        Action resetAction = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, parkPose, 1, true), legs),
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(resetArm, arm));
        Actions.runBlocking(resetAction);
    }
}