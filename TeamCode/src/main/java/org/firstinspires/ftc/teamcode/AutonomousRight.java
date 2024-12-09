package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        Pose2d dropAndPickup = new Pose2d(new Vector2d(12, -40), Math.toRadians(-135));
        Pose2d secondHang = new Pose2d(new Vector2d(23, 2), Math.toRadians(0));
        Pose2d missSub = new Pose2d(new Vector2d(23, -40), Math.toRadians(0));
        Pose2d behind1 = new Pose2d(new Vector2d(40, -50), Math.toRadians(180));
        Pose2d push1 = new Pose2d(new Vector2d(8, -50), Math.toRadians(180));
        Pose2d behind2 = new Pose2d(new Vector2d(40, -50), Math.toRadians(180));
        Pose2d push2 = new Pose2d(new Vector2d(8, -60), Math.toRadians(180));
        Pose2d thirdHang = new Pose2d(new Vector2d(23, 4), Math.toRadians(0));
        Pose2d park = new Pose2d(new Vector2d(12, -40), Math.toRadians(-135));

        Action armAndShoulderPickup = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            arm.gotoMin(AUTO_POWER);
            return false;
        };

        Action pickup = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.GROUND);
            hand.grab(800);
            return false;
        };

        Action readyToHang = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
        };

        Action drop = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
        };




        //TO DO: check for sample in hand, if we dont have one then search for another, else
        //Backs up to get out of the way of our alliance
        Pose2d backUpToChannel = new Pose2d(new Vector2d (3.5, 0), Math.toRadians(90));
        Action driveToBackUpChannelAction = legs.moveToAction(AUTO_POWER, backUpToChannel);
        Actions.runBlocking(new CompleteAction(driveToBackUpChannelAction, legs));

        //Drops off the submersible sample
        Pose2d driveToLowBucketDrop = new Pose2d(new Vector2d(5.4,54.6), Math.toRadians(93));
        Action driveToLowBucketDropAction = legs.moveToAction(AUTO_POWER, driveToLowBucketDrop);
        int lowBucketArmPos = 465;
        Action setArmToLowBucketDrop = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, lowBucketArmPos);
            return false;
        };
        Action setShoulderToLowBucketDrop = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.LOW_BUCKET, AUTO_POWER, lowBucketArmPos);
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
        Actions.runBlocking (dropFirstSample);


        //Go back to original point to get out of the way of our alliance
        Pose2d goBackToOriginalPoint = new Pose2d(new Vector2d(3.5,0), Math.toRadians(-90));
        Action goBackToOriginalPointAction = legs.moveToAction(AUTO_POWER, goBackToOriginalPoint, -1);
        Actions.runBlocking(new CompleteAction(goBackToOriginalPointAction, legs));

        //Go to pick up closest red sample
        Pose2d goToPickupColoredSamples = new Pose2d(new Vector2d (19, -40), Math.toRadians(0));
        Action driveToPickupColoredSamplesAction = legs.moveToAction(AUTO_POWER, goToPickupColoredSamples);
        Actions.runBlocking(new CompleteAction(driveToPickupColoredSamplesAction, legs));

        //!!!CHECKPOINT: NEW WORK BELOW!!!


        //Extend arm and pick up closest sample, TWEAK NUMBERS
        Action liftShoulderToPickupCloseSample = telemetryPacket -> {
          shoulder.setPosition(AUTO_POWER, 543);
          return false;
        };
        Action extendArmToPickupCloseSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 900);
            return false;
        };
        Action setWristToPickupCloseSample = telemetryPacket -> {
            hand.setWrist(0.5);
            return false;
        };
        Action closeSampleSetup = new ParallelAction(
                new CompleteAction(liftShoulderToPickupCloseSample, shoulder),
                new CompleteAction(extendArmToPickupCloseSample, arm),
                new CompleteAction(setWristToPickupCloseSample, hand)
        );
        Actions.runBlocking (closeSampleSetup);

        //Pickup Sample
        Action dropShoulder = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.8, 900);
            return false;
        };

        Action grabAction = telemetryPacket -> {
            hand.grab(GRAB_MS);
            return false;
        };
        Action closeSamplePickup = new ParallelAction(
                new CompleteAction(dropShoulder, shoulder),
                new CompleteAction(grabAction, hand)
        );
        Actions.runBlocking (closeSamplePickup);

        Action raiseShoulder = telemetryPacket -> {
            shoulder.setPosition(0.7, 143);
            return false;
        };
        Action retractArm = telemetryPacket -> {
            arm.setPosition(0.7, 0);
            return false;
        };


        //Move to observation (corner trapezoid)
        Pose2d goToObservation = new Pose2d(new Vector2d (19, -46), Math.toRadians(180));
        Action driveToObservation = legs.moveToAction(AUTO_POWER, goToObservation, 1);
        Actions.runBlocking(new CompleteAction(driveToObservation, legs));


        //Put sample in observation
        Action extendArmToDropSample = telemetryPacket -> {
            arm.setPosition(0.7, 1100);
            return false;
        };
        Actions.runBlocking (new CompleteAction(extendArmToDropSample, arm));

        Action dropSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Actions.runBlocking (new CompleteAction(dropSample, hand));


        //Position to grab middle sample
        Pose2d goToObservationAgain = new Pose2d(new Vector2d (19, -50), Math.toRadians(0));
        Action driveToObservationAgain = legs.moveToAction(AUTO_POWER, goToObservationAgain, -1);
        Actions.runBlocking(new CompleteAction(driveToObservationAgain, legs));

        //Extend arm and pick up middle sample
        //COPY FROM ABOVE ISH
        Action liftShoulderToPickupMiddleSample = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 543);
            return false;
        };
        Action extendArmToPickupMiddleSample = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 900);
            return false;
        };
        Action setWristToPickupMiddleSample = telemetryPacket -> {
            hand.setWrist(0.5);
            return false;
        };
        Action middleSampleSetup = new ParallelAction(
                new CompleteAction(liftShoulderToPickupMiddleSample, shoulder),
                new CompleteAction(extendArmToPickupMiddleSample, arm),
                new CompleteAction(setWristToPickupMiddleSample, hand)
        );
        Actions.runBlocking (new CompleteAction(middleSampleSetup, hand));
            //Go to bucket and drop off sample
        //Drops off the sample
        Actions.runBlocking (bucketDropSetup);
        //IF TIME: grab third sample to put in either bucket or submersible(??)
        // Park in observation area(??)

    }
}