package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousLeft extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // If we grabbed a sample from the center, drive and place in lower bucket

        // Cycle from bucket to floor samples

        // Park in climb 1 area

        /*
        // This action will move to the top bucket to drop off a sample
        // First it looks at the robots current position, calculates how it has to move
        // to the bucket and moves there (careful about our partner).
        // While moving it's raising the shoulder,
        // extending the arm.  With arm extended fingers release and we stow for travel.
        // It might make sense to stay out of our partners way by dropping in the lower bucket.
        ParallelAction toBucket = new ParallelAction(
                legs.actionBuilder(legs.pose)
                        //movement code
                        .build(),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        //shoulder.setPosition(topBucket);
                        //arms.setPosition(out);
                        //fingers.release();
                        //arms.setPosition(stow);
                        //shoulder.setPosition(search);
                        return false;
                    }
                }

        );
        Actions.runBlocking(toBucket);

        // This action will determine where we are (might check apiril tags as well)
        // Then move to grab another floor sample, finally stowing for travel.
        ParallelAction floorSample = new ParallelAction(
                legs.actionBuilder(legs.pose)
                        //movement code
                        //moving to the three lines with the samples
                        //may just be turning with no movement
                        .build(),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        //arms.setPosition(out);
                        //fingers.search();
                        //fingers.pickup();
                        //arms.setPosition(stow);
                        return false;
                    }
                }
        );
        Actions.runBlocking(floorSample);

        // Determine where we are and how we have to move to park correctly
        ParallelAction park = new ParallelAction(
                legs.actionBuilder(legs.pose)
                        //movement code
                        //moving to the three lines with the samples
                        //may just be turning with no movement
                        .build(),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        //shoulder.setPosition(hang);
                        return false;
                    }
                }
        );
        Actions.runBlocking(park);
        */
    }
}