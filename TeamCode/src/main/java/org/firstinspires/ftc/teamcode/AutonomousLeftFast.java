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
        Pose2d highBucketDropPose = new Pose2d(new Vector2d(4.5 + X_OFFSET, 49.5 + Y_OFFSET), Math.toRadians(135));
        Pose2d samplePickupPose = new Pose2d(new Vector2d(24.8 + X_OFFSET, 16.2 + Y_OFFSET), Math.toRadians(62.8));
        Pose2d colorCheckPose = new Pose2d(new Vector2d(20.0 + X_OFFSET, Y_OFFSET), Math.toRadians(45));
        Pose2d intermediatePose = new Pose2d(new Vector2d(15 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(135));
        int firstSearchArmPosition = 1900;
        int midHighDropArmPosition = 1000;
        int highBucketDropArmPosition = 2150;
        double lowerShoulderPower = 0.5;
        double waitTime = 0.5;
        int highBucketShoulderPosition = shoulder.getPositionForMode(Shoulder.Mode.HIGH_BUCKET, highBucketDropArmPosition) - 50;

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, highBucketDropArmPosition);
            hand.bucket();
            return false;
        };

        Action extendArmHalfway = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, midHighDropArmPosition);
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

        Action highBucketRaiseAction = telemetryPacket -> {
            shoulder.setPosition(0.9, highBucketShoulderPosition);
            return false;
        };

        // Setting pose to grab first sample
        Action driveToFirstPickup = new CompleteAction (legs.moveToAction(AUTO_POWER, samplePickupPose, false), legs);

        // Extending arm while turning
        Action armOut = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, firstSearchArmPosition);
            return false;
        };

        Action firstWristGrab = telemetryPacket -> {
            hand.setWrist(0.8);
            return false;
        };

        Action setShoulderToSearch = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.SEARCH, 0.8, firstSearchArmPosition);
                    return false;
        };

        Action samplePickupOne = new ParallelAction(
                driveToFirstPickup,
                new CompleteAction(armOut, arm),
                new CompleteAction(setShoulderToSearch, shoulder),
                firstWristGrab
                );

        Action waitBeforePickup = new SequentialAction(
          samplePickupOne,
          new SleepAction(waitTime)
        );

        Actions.runBlocking(
                waitBeforePickup
        );

        Action firstGrab = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.GROUND,lowerShoulderPower, firstSearchArmPosition);
            hand.grab(800);
            return false;
        };

        Actions.runBlocking(
                new CompleteAction(firstGrab, hand)
        );

        Action firstDrop = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, highBucketDropPose), legs),
                new CompleteAction(highBucketRaiseAction, shoulder),
                new CompleteAction(extendArmAction, arm)
        );

        Action dropInBucket = new SequentialAction(
                new SleepAction(waitTime),
                firstDrop,
                new CompleteAction(releaseSample, hand)
        );

        Actions.runBlocking(
                dropInBucket
        );

        // Repeat this 2 times for each floor sample
        for(int i = 0; i < 2; i++)
        {
            double wristAngle = (i==0) ? 0.5 : 0.65;
            int searchPosition = (i==0) ? 1500 : 1620;

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

            // Turn to ground samples, pick one up
            double turnAngle = (i==0) ? 3.2 : 26.2;
            Pose2d pickupPose = new Pose2d(new Vector2d(15 + X_OFFSET, 47.5 + Y_OFFSET), Math.toRadians(turnAngle));
            Action turnToPickup = legs.moveToAction(0.7, pickupPose, 1);

            Action lower = new ParallelAction(
                    new CompleteAction(retractForPickupAction, arm),
                    new CompleteAction(lowerAction, shoulder)
            );
            Action pauseAndLower = new SequentialAction(
                    new SleepAction(0.35),
                    lower
            );

            Action grabAction = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.GROUND,lowerShoulderPower, searchPosition);
                hand.grab(GRAB_MS);
                return false;
            };
            Action turnAndLower = new ParallelAction(
                    wristAction,
                    new CompleteAction(turnToPickup, legs),
                    pauseAndLower
            );
            Action pickupAction = new SequentialAction(
                    turnAndLower,
                    new SleepAction(waitTime),
                    new CompleteAction(grabAction, hand));
            Actions.runBlocking(pickupAction);

            // Turn to bucket from whatever position we ended up, drop sample in bucket
            Action turnToBucket = legs.moveToAction(0.8, intermediatePose, -1);
            Action turnRaiseAndExtend = new ParallelAction(
                    new CompleteAction(highBucketRaiseAction, shoulder),
                    new CompleteAction(turnToBucket, legs),
                    new CompleteAction(extendArmHalfway, arm));

            Action driveToBucket = legs.moveToAction(0.4, highBucketDropPose,-1);

            Action finalToBucket = new ParallelAction(
                    new CompleteAction(driveToBucket, legs),
                    new CompleteAction(extendArmAction, arm)
            );

            Action dropAction = new SequentialAction(
                    turnRaiseAndExtend,
                    finalToBucket,
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

        Pose2d parkPose1 = new Pose2d(new Vector2d(32.5+X_OFFSET, 30+Y_OFFSET), Math.toRadians(90));
        Pose2d parkPose2 = new Pose2d(new Vector2d(53+X_OFFSET, 32+Y_OFFSET), Math.toRadians(90));
        Pose2d parkPose3 = new Pose2d(new Vector2d(53+X_OFFSET, 12.5+Y_OFFSET), Math.toRadians(90));

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