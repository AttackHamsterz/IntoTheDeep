package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.ArrayList;
import java.util.List;

//extends standard setup op mode
public class AutonomousOpMode extends StandardSetupOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Every autonomous will drive forward, lift the shoulder, and
        // extend the arms at the same time.  Once the arms are
        // extended the shoulder is dropped slightly until the sample
        // is latched onto the sample bar.  Then the arms are retracted
        // while the fingers release the sample.
        Action initialAction = telemetryPacket -> {
            hand.grab(200);
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
            return false;
        };
        Action highbarAction = telemetryPacket -> {
            arm.setPosition(0.9,1000);
            return false;
        };
        Action rotateSampleAction = telemetryPacket -> {
            hand.rotate(750, false);
            return false;
        };
        Action dropAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            return false;
        };

        Action dropToolAction = new SequentialAction(
                initialAction,
                new SleepAction(0.2),
                highbarAction,
                new SleepAction(1.0),
                rotateSampleAction,
                new SleepAction(2.0),
                dropAction,
                new SleepAction(1.5));
        Action dropDriveAction = legs.actionBuilder(startPose)
                .waitSeconds(0.2)
                .lineToX(20)
                .build();

        ParallelAction dropSample = new ParallelAction(dropToolAction, dropDriveAction);
        Actions.runBlocking(dropSample);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        // TODO
    }
}
