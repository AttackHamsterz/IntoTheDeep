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
        Pose2d lowBucketDropPose = new Pose2d(new Vector2d(24, 40), Math.toRadians(170.0));
        int bucketDropArmPosition = 1600;
        int retractArmPosition = 200;
        int searchArmPosition = 800;

        Action driveToLowBucketDrop = legs.moveToAction(AUTO_POWER, lowBucketDropPose);

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, bucketDropArmPosition);
            hand.bucket();
            return false;
        };
        Action raiseArmAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.LOW_BUCKET, AUTO_POWER, bucketDropArmPosition);
            return false;
        };
        Action driveAndExtendAction = new ParallelAction(
            new CompleteAction(driveToLowBucketDrop, legs),
            new CompleteAction(raiseArmAction, shoulder),
            new CompleteAction(extendArmAction, arm)
        );
        Action releaseSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Action retractForPickupAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, retractArmPosition);
            return false;
        };

        Action binDrop = new SequentialAction(
                driveAndExtendAction,
                new CompleteAction(releaseSample, hand),
                retractForPickupAction);
        Actions.runBlocking(binDrop);

        // Cycle from bucket to floor samples
        Action lowerAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, searchArmPosition);
            return false;
        };

        Action extendArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, searchArmPosition);
            return false;
        };

        Action grabAction = telemetryPacket -> {
            // TODO - Search
            hand.grab(GRAB_MS);
            shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.8, searchArmPosition);
            return false;
        };

        Action raiseAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.LOW_BUCKET, AUTO_POWER, bucketDropArmPosition);
            return false;
        };

        // Repeat this 3 times for each floor sample
        /*
        for(int i = 0; i < 3; i++)
        {
            // Turn to ground samples, pick one up
            Pose2d pickupPose = new Pose2d(new Vector2d(24, 40), Math.toRadians(i*10));
            Action turnToPickup = legs.moveToAction(AUTO_POWER, pickupPose);
            Action turnAndLower = new ParallelAction(
                    new CompleteAction(turnToPickup, legs),
                    new CompleteAction(lowerAction, shoulder)
            );
            Action pickupAction = new SequentialAction(
                    turnAndLower,
                    new CompleteAction(extendArm, arm),
                    new CompleteAction(grabAction, hand));
            Actions.runBlocking(pickupAction);

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(AUTO_POWER, lowBucketDropPose);
            Action turnRaiseAndExtend = new ParallelAction(
                    new CompleteAction(turnToBucket, legs),
                    new CompleteAction(raiseAction, shoulder),
                    new CompleteAction(extendArmAction, arm));
            Action dropAction = new SequentialAction(
                    turnRaiseAndExtend,
                    new CompleteAction(releaseSample, hand),
                    retractForPickupAction
            );
            Actions.runBlocking(dropAction);
        }

         */

        // TODO - Park in climb 1 area touching the bar
        // We are facing the buckets and we just dropped a sample
        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetArm = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetAction = new ParallelAction(
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(resetArm, arm));
        Actions.runBlocking(resetAction);
    }
}