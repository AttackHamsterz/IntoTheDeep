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
public class AutonomousOpMode extends StandardSetupOpMode{

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

        double initialWait = 0.25;
        Action hangSampleToolAction = new SequentialAction(
                liftShoulderAction,
                new SleepAction(initialWait),
                extendArmAction,
                new SleepAction(0.5),
                extraGrabAction,
                new SleepAction(1.0),
                dropAction,
                new SleepAction(0.5));
        Action hangSampleDriveAction = new SequentialAction(
                new SleepAction(initialWait),
                legs.moveToAction(new Pose2d(new Vector2d(26, 0), 0)));

        ParallelAction dropSample = new ParallelAction(hangSampleToolAction, hangSampleDriveAction);
        Actions.runBlocking(dropSample);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        Action retractArmAction = telemetryPacket -> {
            arm.setPosition(0.9, 300);
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
            hand.grab(1000);
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
                retractArmAction,
                new SleepAction(1),
                searchAction,
                extendAction,
                new SleepAction(1),
                pickupAction,
                new SleepAction(1),
                searchAction,
                new SleepAction(0.5),
                fullRetract,
                new SleepAction(0.5),
                raiseArmAction,
                new SleepAction(0.5)
        );
        Actions.runBlocking(grabFromSubmersible);
    }
}
