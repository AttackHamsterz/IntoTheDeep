package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// When writing autonomous mode actions, make sure they build very quickly.
// That basically means don't have any sleeps directly in the action.
// So for instance if you need to raise the shoulder, and then release that
// would require a pause.  THe action list would have the pause added to it rather
// than the individual action.  Eventually we should use an event driven architecture!
public class AutonomousOpMode extends StandardSetupOpMode {
    public static final long GRAB_MS = 900;
    public static final long RELEASE_MS = 900;
    public static final double AUTO_POWER = 1.0;

    protected static int dropShoulderPositionTop = 2050;
    protected static int dropShoulderPositionBottom = 1450;
    protected static int dropArmPosition = 960;

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
        int dropArmPullin = 320;
        int searchArmPosition = 1070;
        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, dropShoulderPositionTop);
            return false;
        };
        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER,dropArmPosition);
            hand.hangSample();
            return false;
        };
        Pose2d dropPose = new Pose2d(new Vector2d(22.75, 0), 0);
        Action liftExtendDrive = new ParallelAction(
                new CompleteAction(liftShoulderAction, shoulder), // Shoulder to bar drop position
                new CompleteAction(extendArmAction, arm),         // Arm to bar drop position
                new CompleteAction(legs.moveToAction(0.4, dropPose), legs));


        Action dropAction = telemetryPacket -> {
            hand.grab(400);
            shoulder.setPosition(AUTO_POWER, dropShoulderPositionBottom);
            return false;
        };
        Action retractArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, dropArmPullin);
            return false;
        };

        Action hangSampleToolAction = new SequentialAction(
                liftExtendDrive,                             // Drive and extend
                new CompleteAction(dropAction, shoulder),    // Run dropAction
                new CompleteAction(retractArmAction, arm));
        Actions.runBlocking(hangSampleToolAction);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        boolean doSearch = false;
        if(doSearch) {
            Pose2d searchPose = new Pose2d(new Vector2d(27.0, 0), 0);
            Action searchAction = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, searchArmPosition);
                return false;
            };
            Action extendAction = telemetryPacket -> {
                arm.setPosition(AUTO_POWER, searchArmPosition);
                return false;
            };
            Action pickupAction = telemetryPacket -> {
                hand.grab(GRAB_MS);
                shoulder.setPositionForMode(Shoulder.Mode.GROUND, AUTO_POWER, searchArmPosition);
                return false;
            };
            Action liftShoulder = telemetryPacket -> {
                shoulder.setPosition(AUTO_POWER, 550);
                return false;
            };
            Action retractFromPickup = telemetryPacket -> {
                arm.setPosition(AUTO_POWER, 100);
                legs.moveToPose(AUTO_MOVE_POWER, dropPose);
                return false;
            };

            Action grabFromSubmersible = new SequentialAction(
                    new CompleteAction(searchAction, shoulder), // Search height
                    legs.moveToAction(AUTO_MOVE_POWER, searchPose),
                    new CompleteAction(extendAction, arm),      // Extend arm
                    new CompleteAction(pickupAction, hand),     // Pickup sample
                    new CompleteAction(liftShoulder, shoulder),
                    new CompleteAction(retractFromPickup, arm));          // Retract arm
            Actions.runBlocking(grabFromSubmersible);
        }
    }
}
