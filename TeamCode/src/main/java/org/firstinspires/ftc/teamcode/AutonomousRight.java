package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.concurrent.CopyOnWriteArrayList;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        //TO DO: check for sample in hand, if we dont have one then search for another, else
        boolean haveBlock = camera.blockCaptured();

        //Backs up to get out of the way of our alliance
        Pose2d backUpToChannel = new Pose2d(new Vector2d (3.5, 0), Math.toRadians(90));
        Action driveToBackUpChannelAction = legs.moveToAction(AUTO_POWER, backUpToChannel);
        Actions.runBlocking(new CompleteAction(driveToBackUpChannelAction, legs));

        //Drops off the submersible sample
        Pose2d driveToLowBucketDrop = new Pose2d(new Vector2d(3.5,56.5), Math.toRadians(110));
        Action driveToLowBucketDropAction = legs.moveToAction(AUTO_POWER, driveToLowBucketDrop);
        int lowBucketArmPos = 1200;
        Action setArmToLowBucketDrop = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, lowBucketArmPos);
            return false;
        };

        Action setShoulderToLowBucketDrop = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.LOW_BUCKET);
            return false;
        };

        Action bucketDropSetup = new ParallelAction(
                new CompleteAction(driveToLowBucketDropAction, legs),
                new CompleteAction(setArmToLowBucketDrop, arm),
                new CompleteAction(setShoulderToLowBucketDrop, shoulder)
        );
        Actions.runBlocking (bucketDropSetup);

        Action dropFirstSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Actions.runBlocking (new CompleteAction(dropFirstSample, hand));

        //Go back to original point to get out of the way of our alliance
        Pose2d goBackToOriginalPoint = new Pose2d(new Vector2d(5,0), Math.toRadians(-45));
        Action goBackToOriginalPointAction = legs.moveToAction(AUTO_POWER, goBackToOriginalPoint, -1);
        Action retractArm = telemetryPacket -> {
            arm.setPosition(AUTO_POWER,465);
            return false;
        };
        Action shoulderDown = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(AUTO_POWER, 543);
            return false;
        };
        Action armAndShoulder = new ParallelAction(
                new CompleteAction(retractArm, arm),
                new CompleteAction(shoulderDown, shoulder)
        );
        Action waitToReset = new SequentialAction(
                new SleepAction(0.5),
                armAndShoulder
        );
        Action goBackToOriginalPointArmAndShoulder = new ParallelAction(
                new CompleteAction(goBackToOriginalPointAction, legs),
                waitToReset
                );
        Actions.runBlocking(new CompleteAction(goBackToOriginalPointArmAndShoulder, legs));

        //Go to pick up closest colored sample
        Pose2d goToPickupColoredSamples = new Pose2d(new Vector2d (19, -39.5), Math.toRadians(0));
        Action driveToPickupColoredSamplesAction = legs.moveToAction(AUTO_POWER, goToPickupColoredSamples);
        Actions.runBlocking(new CompleteAction(driveToPickupColoredSamplesAction, legs));
        //!!!CHECKPOINT: NEW WORK BELOW!!!


        //Extend arm and pick up closest sample, TWEAK NUMBERS
        Action extendArmToPickupCloseSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 800);
            hand.setWrist(0.5);
            return false;
        };

        Action closeSampleSetup = new SequentialAction(
                new CompleteAction(extendArmToPickupCloseSample, arm)
        );
        Actions.runBlocking (closeSampleSetup);

        //Pickup Sample
        Action dropShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 143);
            return false;
        };

        Action grabAction = telemetryPacket -> {
            hand.grab(GRAB_MS);
            return false;
        };
        Action liftShoulderToSearch = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 543);;
            return false;
        };
        Action closeSamplePickup = new SequentialAction(
                new CompleteAction(dropShoulder, shoulder),
                new CompleteAction(grabAction, hand),
                new CompleteAction(liftShoulderToSearch, shoulder)
        );
        Actions.runBlocking (closeSamplePickup);


        //Move to observation (corner trapezoid)
        Pose2d tempCodeOne = legs.getPose();
        Pose2d goToObservation = new Pose2d(new Vector2d (19, -39), Math.toRadians(Math.toDegrees(tempCodeOne.heading.toDouble())-180));
        Action driveToObservation = legs.moveToAction(AUTO_POWER, goToObservation, 1);
        Actions.runBlocking(new CompleteAction(driveToObservation, legs));


        //Put sample in observation

        Action liftShoulderToDropSample = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 543);;
            return false;
        };
        Action extendArmToDropSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 1000);
            return false;
        };
        Action setArmAndShoulderForDrop = new SequentialAction(
                new CompleteAction(liftShoulderToDropSample, shoulder),
                new CompleteAction(extendArmToDropSample, arm)
        );
        Actions.runBlocking (new CompleteAction(setArmAndShoulderForDrop, arm));
        Action dropSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Actions.runBlocking (new CompleteAction(dropSample, hand));


        //Position to grab middle sample
        Pose2d goToObservationAgain = new Pose2d(new Vector2d (19, -50), Math.toRadians(10));
        Action driveToObservationAgain = legs.moveToAction(AUTO_POWER, goToObservationAgain, -1);
        Action retractArmForSecondSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 800);
            return false;
        };
        Action goToObservationAgainAction = new ParallelAction(
                new CompleteAction(driveToObservationAgain, legs),
                new CompleteAction(retractArmForSecondSample, arm)
        );
        Actions.runBlocking(new CompleteAction(goToObservationAgainAction, legs));

        //NEW COPIED STUFF BELOW

        Action extendArmToPickupMiddleSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 800);
            hand.setWrist(0.5);
            return false;
        };

        Action middleSampleSetup = new SequentialAction(
                new CompleteAction(extendArmToPickupMiddleSample, arm)
        );
        Actions.runBlocking (middleSampleSetup);

        //Pickup Sample
        Action dropShoulderMid = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER,543);
            return false;
        };

        Action grabActionMid = telemetryPacket -> {
            hand.grab(GRAB_MS);
            return false;
        };
        Action liftShoulderToSearchMid = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 543);
            return false;
        };
        Action middleSamplePickup = new SequentialAction(
                new CompleteAction(dropShoulderMid, shoulder),
                new CompleteAction(grabActionMid, hand),
                new CompleteAction(liftShoulderToSearchMid, shoulder)
        );
        Actions.runBlocking (middleSamplePickup);


        //Move to observation (corner trapezoid)
        Pose2d tempCodeTwo = legs.getPose();
        Pose2d goToObservationMid = new Pose2d(new Vector2d (19, -50), Math.toRadians(Math.toDegrees(tempCodeTwo.heading.toDouble())-180));
        Action driveToObservationMid = legs.moveToAction(AUTO_POWER, goToObservationMid, 1);
        Actions.runBlocking(new CompleteAction(driveToObservationMid, legs));


        //Put sample in observation

        Action liftShoulderToDropSampleMid = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 543);
            return false;
        };
        Action extendArmToDropSampleMid = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 1000);
            return false;
        };
        Action setArmAndShoulderForDropMid = new SequentialAction(
                new CompleteAction(liftShoulderToDropSampleMid, shoulder),
                new CompleteAction(extendArmToDropSampleMid, arm)
        );
        Actions.runBlocking (new CompleteAction(setArmAndShoulderForDropMid, arm));
        Action dropSampleMid = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Actions.runBlocking (new CompleteAction(dropSampleMid, hand));

        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };

        Pose2d parkPose = new Pose2d(new Vector2d(19, 50), Math.toRadians(-45));
        Action resetAction = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, parkPose, 1), legs),
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(resetArm, arm));
        Actions.runBlocking(resetAction);
    }
}