package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        //TO DO: check for sample in hand, if we dont have one then search for another, else
        //Backs up to get out of the way of our alliance
        Pose2d backUpToChannel = new Pose2d(new Vector2d (3.5, 0), Math.toRadians(90));
        Action driveToBackUpChannelAction = legs.moveToAction(AUTO_POWER, backUpToChannel);
        Actions.runBlocking(new CompleteAction(driveToBackUpChannelAction, legs));

        //Drops off the submersible sample
        Pose2d driveToLowBucketDrop = new Pose2d(new Vector2d(5.4,65), Math.toRadians(135));
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
        Actions.runBlocking (new CompleteAction(dropFirstSample, hand));

        //Go back to original point to get out of the way of our alliance
        Pose2d goBackToOriginalPoint = new Pose2d(new Vector2d(3.5,0), Math.toRadians(-90));
        Action goBackToOriginalPointAction = legs.moveToAction(AUTO_POWER, goBackToOriginalPoint, -1);
        Actions.runBlocking(new CompleteAction(goBackToOriginalPointAction, legs));

        //Go to pick up closest red sample
        Pose2d goToPickupColoredSamples = new Pose2d(new Vector2d (19, -39), Math.toRadians(0));
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
            hand.setWrist(0.5);
            return false;
        };

        Action closeSampleSetup = new ParallelAction(
                new CompleteAction(liftShoulderToPickupCloseSample, shoulder),
                new CompleteAction(extendArmToPickupCloseSample, arm)
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
        Action closeSamplePickup = new SequentialAction(
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
        Pose2d tempCodeOne = legs.getPose();
        Pose2d goToObservation = new Pose2d(new Vector2d (19, -39), Math.toDegrees(tempCodeOne.heading.toDouble()-180));
        Action driveToObservation = legs.moveToAction(AUTO_POWER, goToObservation, 1);
        Actions.runBlocking(new CompleteAction(driveToObservation, legs));


        //Put sample in observation
        Action dropSample = telemetryPacket -> {
            hand.release(RELEASE_MS);
            return false;
        };
        Actions.runBlocking (new CompleteAction(dropSample, hand));

        Action liftArmToDropSample = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            return false;
        };
        Actions.runBlocking (new CompleteAction(liftArmToDropSample, shoulder));


        //Position to grab middle sample
        Pose2d tempCodeTwo = legs.getPose();
        Pose2d goToObservationAgain = new Pose2d(new Vector2d (19, -50), Math.toDegrees(tempCodeTwo.heading.toDouble()+180));
        Action driveToObservationAgain = legs.moveToAction(AUTO_POWER, goToObservationAgain, -1);
        Actions.runBlocking(new CompleteAction(driveToObservationAgain, legs));

        //Extend arm and pick up middle sample
        //COPY FROM ABOVE ISH
        //Action liftShoulderToPickupMiddleSample = telemetryPacket -> {
            //shoulder.setPosition(AUTO_POWER, 543);
            //return false;
        //};
        //Action extendArmToPickupMiddleSample = telemetryPacket -> {
            //arm.setPosition(AUTO_POWER, 900);
            //return false;
        //};
        //Action setWristToPickupMiddleSample = telemetryPacket -> {
            //hand.setWrist(0.5);
            //return false;
        //};
        //Action middleSampleSetup = new ParallelAction(
                //new CompleteAction(liftShoulderToPickupMiddleSample, shoulder),
                //new CompleteAction(extendArmToPickupMiddleSample, arm),
                //new CompleteAction(setWristToPickupMiddleSample, hand)
        //);
        //Actions.runBlocking (new CompleteAction(middleSampleSetup, hand));
            //Go to bucket and drop off sample
        //Drops off the sample
        //Actions.runBlocking (bucketDropSetup);
        //IF TIME: grab third sample to put in either bucket or submersible(??)
        // Park in observation area(??)

    }
}