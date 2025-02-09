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
    public static final long GRAB_MS = 900;
    public static final long RELEASE_MS = 900;
    public static final double AUTO_POWER = 1.0;

    protected static int dropArmPosition = 270;

    protected boolean submersibleSearch = false;
    protected boolean partnerHasSpecimen = false;

    // If the field is asymmetric offset poses' (inches)
    // Offsets are considered positive for blue and negative for red
    // For instance Blue at badger bots is a quarter inch shorter
    protected double X_OFFSET = 0;
    protected double Y_OFFSET = 0;

    public void partnerHasSpecimen(){
        this.partnerHasSpecimen = true;
    }

    // TODO - never search
    public void searchSubmersible(){
        //this.submersibleSearch = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Remove the floor protection (causing some position override issues in autonomous)
        //arm.setShoulder(null);

        // Flip offsets if red (asymmetric field)
        if(color == COLOR.RED){
            X_OFFSET = -X_OFFSET;
            Y_OFFSET = -Y_OFFSET;
        }

        // Every autonomous will drive forward, lift the shoulder, and
        // extend the arms at the same time.  Once the arms are
        // extended the shoulder is dropped slightly until the sample
        // is latched onto the sample bar.  Then the arms are retracted
        // while the fingers release the sample.
        int searchArmPosition = 1250;

        Pose2d dropPose = new Pose2d(new Vector2d(31 + X_OFFSET, Y_OFFSET), 0);
        Pose2d searchPose = new Pose2d(new Vector2d(22.0 + X_OFFSET, Y_OFFSET), 0);

        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
            return false;
        };
        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(1.0, 270);
            hand.hangSample();
            return false;
        };
        Action dropAction = telemetryPacket -> {
            //hand.grab(100);
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(AUTO_POWER, Shoulder.DROP_SHOULDER_POS);
            return false;
        };
        Action retractArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action releaseAction = telemetryPacket -> {
            hand.release(600);
            return false;
        };


        Action waitAction = new SequentialAction(
          new SleepAction(0.1),
                new CompleteAction(liftShoulderAction, shoulder) // Shoulder to bar drop position
        );



        Action liftExtendDrive = new ParallelAction(
                waitAction,
                //new CompleteAction(liftShoulderAction, shoulder), // Shoulder to bar drop position
                new CompleteAction(extendArmAction, arm),         // Arm to bar drop position
                new CompleteAction(legs.moveToAction(0.6, dropPose), legs));
        Actions.runBlocking(liftExtendDrive);

        // Extra alignment
        Action action = eye.safeHang();
        if(action != null)
            Actions.runBlocking(action);

        Action retractReleaseBackup = new ParallelAction(
                new CompleteAction(retractArmAction, arm),
                new CompleteAction(releaseAction, hand),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, true), legs)
        );

        Action hangSampleToolAction = new SequentialAction(                 // Drive and extend
                new CompleteAction(dropAction, shoulder),    // Run dropAction
                retractReleaseBackup);
        Actions.runBlocking(hangSampleToolAction);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        if(submersibleSearch) {
            Action searchAction = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, searchArmPosition);
                return false;
            };
            Action extendAction = telemetryPacket -> {
                shoulder.setMode(Shoulder.Mode.SEARCH);
                arm.setPosition(AUTO_POWER, searchArmPosition);
                return false;
            };
            Action setupSearch = new SequentialAction(
                    searchAction, // Search height
                    new CompleteAction(extendAction, arm));
            Actions.runBlocking(setupSearch);

            // Real Search (should block until complete)
            Action safeSearch = eye.safeSearch();
            shoulder.setMode(Shoulder.Mode.NONE);
            Actions.runBlocking(safeSearch);

            // Safe height for shoulder
            Action liftShoulder = telemetryPacket -> {
                shoulder.setPosition(AUTO_POWER, 406);
                return false;
            };
            Actions.runBlocking(new CompleteAction(liftShoulder, shoulder));

            Action retractBackup = new ParallelAction(
                    new CompleteAction(retractArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, true), legs)
            );

            Actions.runBlocking(retractBackup);
        }
    }
}
