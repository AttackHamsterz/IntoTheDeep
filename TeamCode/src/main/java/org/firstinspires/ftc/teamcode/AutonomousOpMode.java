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
    public static final double AUTO_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Remove the floor protection (causing some position override issues in autonomous)
        arm.setShoulder(null);

        // Every autonomous will drive forward, lift the shoulder, and
        // extend the arms at the same time.  Once the arms are
        // extended the shoulder is dropped slightly until the sample
        // is latched onto the sample bar.  Then the arms are retracted
        // while the fingers release the sample.
        int dropArmPosition = 900;
        int searchArmPosition = 1500;
        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.HIGH_BAR, AUTO_POWER, dropArmPosition);
            return false;
        };
        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER,dropArmPosition);
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
        Action driveAndExtend = new ParallelAction(
                new CompleteAction(extendArmAction, arm),   // Extend arm for sample hang
                legs.moveToAction(new Pose2d(new Vector2d(26, 0), 0))
        );
        Action hangSampleToolAction = new SequentialAction(
                liftShoulderAction,                         // Start shoulder lift to avoid dragging
                new SleepAction(initialWait),               // Sleep a little (avoid dragging)
                extraGrabAction,                            // Tighten sample in grip
                driveAndExtend,                             // Drive and extend
                new CompleteAction(dropAction, shoulder));  // Run dropAction
        Actions.runBlocking(hangSampleToolAction);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        Action retractArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 400);
            return false;
        };
        Action searchAction = telemetryPacket -> {
            shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, searchArmPosition);
            return false;
        };
        Action extendAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 1500);
            return false;
        };
        Action pickupAction = telemetryPacket -> {
            hand.grab(GRAB_MS);
            shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.8, searchArmPosition);
            return false;
        };
        Action retract = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 250);
            arm.setPosition(AUTO_POWER, 400);
            return false;
        };

        Action grabFromSubmersible = new SequentialAction(
                retractArmAction,                           // Pull your arm in
                new CompleteAction(searchAction, shoulder), // Search
                new CompleteAction(extendAction, arm),      // Extend arm
                pickupAction,                               // Pickup sample
                new SleepAction(GRAB_S),                    // Wait for hand to pickup (might not reach ground)
                new CompleteAction(retract, arm));          // Retract arm
        Actions.runBlocking(grabFromSubmersible);
    }
}
