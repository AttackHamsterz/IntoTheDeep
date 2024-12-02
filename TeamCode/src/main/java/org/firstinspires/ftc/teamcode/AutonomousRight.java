package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        //TO DO: check for sample in hand
        //Backs up to get out of the way of our alliance
        Pose2d backUpToChannel = new Pose2d(new Vector2d (3.5, 0), Math.toRadians(90));
        Action driveToBackUpChannelAction = legs.moveToAction(AUTO_POWER, backUpToChannel);
        Actions.runBlocking(new CompleteAction(driveToBackUpChannelAction, legs));

        //Drops off the submersible sample
        Pose2d driveToLowBucketDrop = new Pose2d(new Vector2d(3.5,45), Math.toRadians(105));
        Action driveToLowBucketDropAction = legs.moveToAction(AUTO_POWER, driveToLowBucketDrop);
        Actions.runBlocking(new CompleteAction(driveToLowBucketDropAction,legs));

        //Go back to original point to get out of the way of our alliance
        Pose2d goBackToOriginalPoint = new Pose2d(new Vector2d(3.5,0), Math.toRadians(-90));
        Action goBackToOriginalPointAction = legs.moveToAction(AUTO_POWER, goBackToOriginalPoint, -1);
        Actions.runBlocking(new CompleteAction(goBackToOriginalPointAction, legs));

        //Go to pick up closest red sample
        Pose2d goToPickupColoredSamples = new Pose2d(new Vector2d (19, -40), Math.toRadians(0));
        Action driveToPickupColoredSamplesAction = legs.moveToAction(AUTO_POWER, goToPickupColoredSamples);
        Actions.runBlocking(new CompleteAction(driveToPickupColoredSamplesAction, legs));

        //!!!CHECKPOINT: NEW WORK BELOW!!!


        //Extend arm and pick up closest sample


        //Move and put red sample in observation (corner trapezoid)
        Pose2d goToObservation = new Pose2d(new Vector2d (19, -46), Math.toRadians(160));
        Action driveToObservation = legs.moveToAction(AUTO_POWER, goToObservation, 1);
        Actions.runBlocking(new CompleteAction(driveToObservation, legs));

        //Position to grab middle sample


        //Extend arm and pick up middle sample


        //Go to submersible and drop off sample

        //IF TIME: grab third sample to put in either bucket or submersible(??)
        // Park in observation area(??)

    }
}