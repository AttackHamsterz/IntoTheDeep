package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
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
        double bucketDropAngle = 170.0;
        Pose2d lowBucketDropPose = new Pose2d(new Vector2d(26, 40), Math.toRadians(bucketDropAngle));
        Action driveToLowBucketDrop = legs.moveToAction(lowBucketDropPose);

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(0.9, 1700);
            hand.bucket();
            return false;
        };
        Action releaseSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Action retractForPickupAction = telemetryPacket -> {
            arm.setPosition(0.9, 500);
            return false;
        };

        Action binDrop = new SequentialAction(
                driveToLowBucketDrop,
                new CompleteAction(extendArmAction, arm),
                releaseSample,
                new SleepAction(RELEASE_S),
                retractForPickupAction);
        Actions.runBlocking(binDrop);

        // Cycle from bucket to floor samples
        Action lowerAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            return false;
        };

        Action extendArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(0.9, 800);
            return false;
        };

        Action grabAction = telemetryPacket -> {
            // TODO - Search
            hand.grab(GRAB_MS);
            shoulder.setMode(Shoulder.Mode.GROUND);
            return false;
        };

        Action raiseAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.LOW_BUCKET);
            return false;
        };

        // Repeat this 3 times for each floor sample
        /*
        for(int i = 0; i < 3; i++)
        {
            // Turn to ground samples, pick one up
            Action turnToPickup = legs.actionBuilder(legs.getPose())
                    .turnTo(Math.toRadians(i*5))
                    .build();
            Action pickupAction = new SequentialAction(
                    turnToPickup,
                    lowerAction,
                    new CompleteAction(extendArm, arm),
                    grabAction,
                    new SleepAction(GRAB_S),
                    new CompleteAction(raiseAction, shoulder));
            Actions.runBlocking(pickupAction);

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(lowBucketDropPose);
            Action dropAction = new SequentialAction(
                    turnToBucket,
                    new CompleteAction(extendArmAction, arm),
                    releaseSample,
                    new SleepAction(RELEASE_S),
                    retractForPickupAction
            );
            Actions.runBlocking(dropAction);
        }

         */

        // TODO - Park in climb 1 area touching the bar
        // We are facing the buckets and we just dropped a sample

    }
}