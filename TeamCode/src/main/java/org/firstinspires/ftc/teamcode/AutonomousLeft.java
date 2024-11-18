package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousLeft extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // If we grabbed a sample from the center, drive and place in lower bucket
        double bucketDropAngle = 170.0;
        Pose2d lowBucketDropPose = new Pose2d(new Vector2d(26, 40), Math.toRadians(bucketDropAngle));
        Action driveTolowBucketDrop = legs.moveToAction(lowBucketDropPose);

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(0.9, 1700);
            hand.bucket();
          return false;
        };

        Action releaseSample = telemetryPacket -> {
            hand.release(1000);
            return false;
        };

        Action retractForPickupAction = telemetryPacket -> {
            arm.setPosition(0.9, 500);
            return false;
        };

        Action toBin = new SequentialAction(
                driveTolowBucketDrop,
                extendArmAction,
                new SleepAction(2),
                releaseSample,
                new SleepAction(1),
                retractForPickupAction);
        Actions.runBlocking(toBin);

        // Cycle from bucket to floor samples
        Action lowerShoulder = telemetryPacket -> {
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
            hand.grab(1000);
            shoulder.setMode(Shoulder.Mode.GROUND);
            return false;
        };

        Action raiseAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.LOW_BUCKET);
            return false;
        };

        // Repeat this 3 times for each floorsample
        for(int i = 0; i < 3; i++)
        {
            // Turn to ground samples, pick one up
            Action turnToPickup = legs.actionBuilder(legs.getPose())
                    .turnTo(Math.toRadians(i*5))
                    .build();
            Action pickupAction = new SequentialAction(
                    turnToPickup,
                    extendArm,
                    new SleepAction(0.5),
                    grabAction,
                    new SleepAction(1.0),
                    raiseAction
            );
            Actions.runBlocking(pickupAction);

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(lowBucketDropPose);
            Action dropAction = new SequentialAction(
                    turnToBucket,
                    new SleepAction(1.0),
                    extendArmAction,
                    new SleepAction(2),
                    releaseSample,
                    new SleepAction(1),
                    retractForPickupAction
            );
            Actions.runBlocking(dropAction);
        }

        // TODO - Park in climb 1 area touching the bar
        // We are facing the buckets and we just dropped a sample

    }
}