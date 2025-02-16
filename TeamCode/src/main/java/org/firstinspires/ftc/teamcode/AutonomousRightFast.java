package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRightFast extends AutonomousOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // No overriding mode for shoulder
        shoulder.setMode(Shoulder.Mode.NONE);

        // All the poses for the right side
        Pose2d pickupPose = new Pose2d(new Vector2d(8.3 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));
        Pose2d hangPose1 = new Pose2d(new Vector2d(31 + X_OFFSET, 2 + Y_OFFSET), Math.toRadians(0));
        Pose2d hangPose2 = new Pose2d(new Vector2d(31 + X_OFFSET, 2.5 + Y_OFFSET), Math.toRadians(0));
        Pose2d hangPose3 = new Pose2d(new Vector2d(31 + X_OFFSET, 3 + Y_OFFSET), Math.toRadians(0));
        Pose2d barBackupPose = new Pose2d(new Vector2d(25 + X_OFFSET, 2 + Y_OFFSET), Math.toRadians(0));

        // Actions
        Action grab = telemetryPacket -> {
            hand.grab(500);
            return false;
        };
        Action release = telemetryPacket -> {
            hand.release(600);
            return false;
        };

        // Swivel
        for(int i = 0; i < 3; i++)
        {
            // Variables
            double wristAngle = (i == 0) ? 0.4 : (i ==1) ? 0.2 : 0.1;
            double swivelAngle = (i == 0) ? -33.18 : (i == 1) ? -56.24 : -67.15;
            int armPosition = (i == 0) ? 316 : (i == 1) ? 1200 : 2150;
            double armExtendPower = (i==0) ? 0.5 : (i ==1) ? AUTO_POWER : 0.2;
            Pose2d swivelPickPose = new Pose2d(new Vector2d(25.55, -33.71), Math.toRadians(swivelAngle));

            // pick up sample
            // rotate wrist
            Action rotateWrist = telemetryPacket -> {
                hand.setWrist(wristAngle);
                return false;
            };
            // extend arm
            Action extendArm = telemetryPacket -> {
                arm.setPosition(armExtendPower, armPosition);
                return false;
            };
            Action setShoulderPickup = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, armPosition);
                return false;
            };
            Action moveToPickup = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, swivelPickPose, false), legs),
                rotateWrist,
                new CompleteAction(extendArm, arm),
                new CompleteAction(setShoulderPickup, shoulder)
            );

            //drop shoulder and intake
            Action lowerShoulder = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.GROUND, 0.6, armPosition);
                return false;
            };

            Action grabAction = new ParallelAction(
                lowerShoulder,
                new CompleteAction(grab, hand)
            );

            Action turnAndGrab = new SequentialAction(
                moveToPickup,
                grabAction
            );

            Actions.runBlocking(turnAndGrab);

            int dropArmPosition = 2050;
            Pose2d swivelDropPose = new Pose2d(new Vector2d(25.55, -33.71), Math.toRadians(-156));

            Action extendArmForDrop = telemetryPacket -> {
                arm.setPosition(AUTO_POWER, dropArmPosition);
                return false;
            };
            Action hoverShoulder = telemetryPacket -> {
                shoulder.setPositionForMode(Shoulder.Mode.SEARCH, AUTO_POWER, armPosition);
                return false;
            };

            Action raiseAndRotate = new ParallelAction(
                new CompleteAction(hoverShoulder, shoulder) ,
                new CompleteAction(legs.moveToAction(AUTO_POWER, swivelDropPose, true), legs),
                new CompleteAction(extendArmForDrop, arm)
            );

            // rotate and release sample
            Action dropIt = new SequentialAction(
                raiseAndRotate,
                release //new CompleteAction(release, hand)
            );

            // drop sample
            Actions.runBlocking(dropIt);
        }

        // Cycle specimens onto bar

        // Actions for cycling
        Action rotateWrist = telemetryPacket -> {
            hand.hangSample();
            return false;
        };
        Action armIn = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, -15);
            return false;
        };
        Action hoverShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, Shoulder.Mode.SEARCH.armInPos());
            return false;
        };
        Action ground = telemetryPacket -> {
            shoulder.setPosition(0.8, Shoulder.Mode.GROUND.armInPos());
            return false;
        };
        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
            return false;
        };
        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(0.8, dropArmPosition);
            return false;
        };
        Action drop = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.NONE);
            shoulder.setPosition(AUTO_POWER, Shoulder.DROP_SHOULDER_POS);
            return false;
        };

        for (int i = 0; i < 3; i++) {
            // Do we have enough time to drive back?
            if(runtime.seconds() > 28.0) break;

            // move to grab the specimen
            Action gotoPickup = new ParallelAction(
                    rotateWrist,
                    new CompleteAction(armIn, arm),
                    new CompleteAction(hoverShoulder, shoulder),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, pickupPose, -1), legs));
            Actions.runBlocking(gotoPickup);
            // use camera to adjust position
            Action adjust = eye.safeFloor(2);
            if(adjust != null)
                Actions.runBlocking(adjust);
            // pick up specimen
            Action grabAction = new SequentialAction(
                    ground,
                    new CompleteAction(grab, hand)
            );
            Actions.runBlocking(grabAction);

            // Check if we have enough time to hang (otherwise stay parked)
            if(runtime.seconds() > 26.0) break;

            // move to bar
            // go to different bar positions each time
            Pose2d hangPose = (i == 0) ? hangPose1 : (i == 1) ? hangPose2 : hangPose3;
            Action gotoHang = new ParallelAction(
                    new CompleteAction(liftShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, hangPose, 1), legs)
            );
            Actions.runBlocking(gotoHang);

            // Extra bar alignment
            Action action = eye.safeHang(2);
            if(action != null)
                Actions.runBlocking(action);

            // hook onto bar
            Action retractReleaseBackup = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(release, hand),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, barBackupPose, true), legs)
            );
            Action dropAndRelease = new SequentialAction(
                    new CompleteAction(drop, shoulder),
                    retractReleaseBackup
            );
            Actions.runBlocking(dropAndRelease);
        }
    }
}