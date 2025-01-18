package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        // No overriding mode for shoulder
        shoulder.setMode(Shoulder.Mode.NONE);

        // All the poses for the right side
        Pose2d dropAndPickup = new Pose2d(new Vector2d(8.3 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));
        Pose2d secondHang = new Pose2d(new Vector2d(31 + X_OFFSET, 7 + Y_OFFSET), Math.toRadians(0));
        Pose2d avoidSub = new Pose2d(new Vector2d(22 + X_OFFSET, -24.2 + Y_OFFSET), Math.toRadians(0));
        Pose2d behind1 = new Pose2d(new Vector2d(46 + X_OFFSET, -39 + Y_OFFSET), Math.toRadians(180));
        Pose2d push1 = new Pose2d(new Vector2d(8 + X_OFFSET, -35 + Y_OFFSET), Math.toRadians(180));
        Pose2d behind2 = new Pose2d(new Vector2d(46 + X_OFFSET, -46 + Y_OFFSET), Math.toRadians(180));
        Pose2d push2 = new Pose2d(new Vector2d(10 + X_OFFSET, -50 + Y_OFFSET), Math.toRadians(180));
        Pose2d push2Backup = new Pose2d(new Vector2d(18 + X_OFFSET, -45 + Y_OFFSET), Math.toRadians(180));
        Pose2d safeSpot = new Pose2d(new Vector2d(18 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));
        Pose2d thirdHang = new Pose2d(new Vector2d(32 + X_OFFSET, 9 + Y_OFFSET), Math.toRadians(0));
        Pose2d park = new Pose2d(new Vector2d(8.3 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));

        Pose2d searchPose = new Pose2d(new Vector2d(22.0 + X_OFFSET, Y_OFFSET), 0);

        // Common actions for the right side
        Action hoverShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, Shoulder.Mode.SEARCH.armInPos());
            return false;
        };

        Action armIn = telemetryPacket -> {
            arm.gotoMin(AUTO_POWER);
            return false;
        };

        Action grab = telemetryPacket -> {
            hand.grab(500);
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

        Action release = telemetryPacket -> {
            hand.release(800);
            return false;
        };

        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };

        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };

        // If our partner is not using the alliance color specimen (do second hang)
        double APPROACH_POWER = 0.8;
        if(!partnerHasSpecimen) {
            Action gotoPickup = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(hoverShoulder, shoulder),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, dropAndPickup), legs));
            Actions.runBlocking(gotoPickup);

            Action grabAction = new SequentialAction(
                    ground,
                    new CompleteAction(grab, hand)
            );
            Actions.runBlocking(grabAction);

            // Use if tool is dragging on the ground
            //Actions.runBlocking(new CompleteAction(hoverShoulder, shoulder));

            Action gotoSecondHang = new ParallelAction(
                    new CompleteAction(liftShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, secondHang, 1), legs)
            );
            Actions.runBlocking(gotoSecondHang);

            // Extra alignment
            /*
            Action action = eye.safeHang();
            if(action != null)
                Actions.runBlocking(action);

             */
            Action retractReleaseBackup = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(release, hand),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, false), legs)
            );


            Action dropAndRelease = new SequentialAction(
                    new CompleteAction(drop, shoulder),
                    retractReleaseBackup
            );
            Actions.runBlocking(dropAndRelease);


        }

        Action avoidAndHover = new ParallelAction(
                new CompleteAction(armIn, arm),
                new CompleteAction(hoverShoulder, shoulder)
        );

        Action driveAction = new SequentialAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, avoidSub, true), legs),
                avoidAndHover,
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, push1, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind2, true), legs),
                new CompleteAction(legs.moveToAction(0.8, push2, -1, true), legs),
                new CompleteAction(legs.moveToAction(0.8, push2Backup, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, true), legs),
                new CompleteAction(legs.moveToAction(APPROACH_POWER, dropAndPickup), legs)
        );
        Actions.runBlocking(driveAction);

        Action grab2Action = new SequentialAction(
                ground,
                new CompleteAction(grab, hand)
        );
        Actions.runBlocking(grab2Action);

        Action gotoThirdHang = new ParallelAction(
                new CompleteAction(liftShoulderAction, shoulder),
                new CompleteAction(extendArmAction, arm),
                new CompleteAction(legs.moveToAction(AUTO_POWER, thirdHang, 1), legs)
        );
        Actions.runBlocking(gotoThirdHang);

        // Extra alignment
        /*
        Action action = eye.safeHang();
        if(action != null)
            Actions.runBlocking(action);

         */

        Action retractReleaseBackup2 = new ParallelAction(
                new CompleteAction(armIn, arm),
                new CompleteAction(release, hand),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, false), legs)
        );

        Action dropAndRelease2 = new SequentialAction(
                new CompleteAction(drop, shoulder),
                retractReleaseBackup2
        );

        Actions.runBlocking(dropAndRelease2);

        // If our partner used a specimen we have time for one more hang
        if(partnerHasSpecimen){
            Action finalPickup = new ParallelAction(
                    new CompleteAction(resetArm, arm),
                    new CompleteAction(hoverShoulder, shoulder),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, 1, true), legs),
                    new CompleteAction(legs.moveToAction(APPROACH_POWER, dropAndPickup, 1), legs));
            Actions.runBlocking(finalPickup);

            Action grab3Action = new SequentialAction(
                    ground,
                    new CompleteAction(grab, hand)
            );
            Actions.runBlocking(grab3Action);

            Action repeatSecondHang = new ParallelAction(
                    new CompleteAction(liftShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, secondHang, 1), legs)
            );
            Actions.runBlocking(repeatSecondHang);

            // Extra alignment
            /*
            Action action2 = eye.safeHang();
            if(action2 != null)
                Actions.runBlocking(action2);

             */

            Action retractReleaseBackup3 = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(release, hand),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, false), legs)
            );

            Action dropAndRelease3 = new SequentialAction(
                    new CompleteAction(drop, shoulder),
                    retractReleaseBackup3
            );
            Actions.runBlocking(dropAndRelease3);
        }

        Actions.runBlocking(new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, 1, true), legs));

        Action resetAction = new ParallelAction(
                new CompleteAction(resetArm, arm),
                new CompleteAction(hoverShoulder, shoulder),
                new CompleteAction(legs.moveToAction(AUTO_POWER, park, 1), legs));
        Actions.runBlocking(resetAction);

        Action extraResetAction = new ParallelAction(
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(grab, hand));
        Actions.runBlocking(extraResetAction);
    }
}