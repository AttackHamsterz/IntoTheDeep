package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// When writing autonomous mode actions, make sure they build very quickly.
// That basically means don't have any sleeps directly in the action.
// So for instance if you need to raise the shoulder, and then release that
// would require a pause.  THe action list would have the pause added to it rather
// than the individual action.  Eventually we should use an event driven architecture!
public class AutonomousOpMode extends StandardSetupOpMode {
    public static final long GRAB_MS = 1000;
    public static final double GRAB_S = (double)GRAB_MS / 1000.0;
    public static final long RELEASE_MS = 1000;
    public static final double RELEASE_S = (double)RELEASE_MS / 1000.0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Every autonomous will drive forward, lift the shoulder, and
        // extend the arms at the same time.  Once the arms are
        // extended the shoulder is dropped slightly until the sample
        // is latched onto the sample bar.  Then the arms are retracted
        // while the fingers release the sample.
        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
            return false;
        };
        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(0.9,900);
            hand.hangSample();
            return false;
        };
        Action extraGrabAction = telemetryPacket -> {
            hand.grab(200);
            return false;
        };
        Action dropAction = telemetryPacket -> {
            shoulder.dropSample();
            return false;
        };

        double initialWait = 0.2;
        Action hangSampleToolAction = new SequentialAction(
                liftShoulderAction,                         // Start shoulder lift to avoid dragging
                new SleepAction(initialWait),               // Sleep a little (avoid dragging)
                extraGrabAction,                            // Tighten sample in grip
                new CompleteAction(extendArmAction, arm),   // Extend arm for sample hang
                new CompleteAction(dropAction, shoulder));  // Run dropAction
        Action hangSampleDriveAction = new SequentialAction(
                new SleepAction(0.5),           // Sleep a little (avoid dragging)
                legs.moveToAction(new Pose2d(new Vector2d(26, 0), 0)));

        ParallelAction dropSample = new ParallelAction(hangSampleToolAction, hangSampleDriveAction);
        Actions.runBlocking(dropSample);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        Action retractArmAction = telemetryPacket -> {
            arm.setPosition(0.9, 400);
            return false;
        };
        Action searchAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            return false;
        };
        Action extendAction = telemetryPacket -> {
            arm.setPosition(0.9, 1500);
            return false;
        };
        Action pickupAction = telemetryPacket -> {
            hand.grab(GRAB_MS);
            shoulder.setMode(Shoulder.Mode.GROUND);
            return false;
        };
        Action fullRetract = telemetryPacker -> {
            arm.gotoMin(0.9);
            return false;
        };
        Action raiseArmAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.LOW_BUCKET);
            return false;
        };

        Action grabFromSubmersible = new SequentialAction(
                new CompleteAction(retractArmAction, arm),  // Pull your arm in
                new CompleteAction(searchAction, shoulder), // Search
                new CompleteAction(extendAction, arm),      // Extend arm
                pickupAction,                               // Pickup sample
                new SleepAction(GRAB_S+0.25),                    // Wait for hand to pickup (might not reach ground)
                new CompleteAction(searchAction, shoulder), // Back to search position
                new CompleteAction(fullRetract, arm),       // Retract arm
                raiseArmAction);                            // Raise shoulder so avoids walls
        Actions.runBlocking(grabFromSubmersible);
    }
}
