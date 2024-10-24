package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "Autonomous: Far From Board Blue", group = "Robot")
public class AutonomousBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d (0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            // This action will drive forward, lift the shoulder, and
            // extend the arms at the same time.  Once the arms are
            // extended the shoulder is dropped slightly until the sample
            // is latched onto the sample bar.  Then the arms are retracted
            // while the fingers release the sample.
            ParallelAction sampleDrop = new ParallelAction(
                    drive.actionBuilder(beginPose)
                            .lineToX(20)
                            .build(),
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            //shoulder.setPosition(highSample);
                            return false;
                        }
                    },
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            //arms.setPosition(out);
                            //shoulder.setPosition(highSample+200);
                            //fingers.release(1s); // NON BLOCKING
                            //arms.setPosition(out+200);
                            return false;
                        }
                    }
            );

            // This action will grab a sample from the submersible
            // and then stow for travel (retract the arm and set the shoulder)
            ParallelAction snagSample = new ParallelAction(
                    drive.actionBuilder(drive.pose)
                            .lineToX(25)
                            .build(),
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            //shoulder.setPosition(search);
                            //fingers.search();
                            //fingers.pickup();
                            //arms.setPosition(stow);
                            return false;
                        }
                    }
            );

            // This action will move to the top bucket to drop off a sample
            // First it looks at the robots current position, calculates how it has to move
            // to the bucket and moves there (careful about our partner).
            // While moving it's raising the shoulder,
            // extending the arm.  With arm extended fingers release and we stow for travel.
            // It might make sense to stay out of our partners way by dropping in the lower bucket.
            ParallelAction toBucket = new ParallelAction(

            );

            // This action will determine where we are (might check apiril tags as well)
            // Then move to grab another floor sample, finally stowing for travel.
            ParallelAction floorSample = new ParallelAction(

            );

            // Determine where we are and how we have to move to park correctly
            ParallelAction park = new ParallelAction(

            );

            waitForStart();

            Actions.runBlocking(sampleDrop);
            Actions.runBlocking(snagSample);
            Actions.runBlocking(toBucket);
            Actions.runBlocking(floorSample);
            Actions.runBlocking(toBucket);
            Actions.runBlocking(floorSample);
            Actions.runBlocking(toBucket);
            Actions.runBlocking(floorSample);
            Actions.runBlocking(toBucket);
            Actions.runBlocking(park);
        }
    }

}
