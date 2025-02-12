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
        Pose2d basicHangPose = new Pose2d(new Vector2d(31 + X_OFFSET, 2 + Y_OFFSET), Math.toRadians(0));
        Pose2d barBackupPose = new Pose2d(new Vector2d(25 + X_OFFSET, 2 + Y_OFFSET), Math.toRadians(0));

        // Actions
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
            int armPosition = (i == 0) ? 316 : (i == 1) ? 1200 : 2185;
            Pose2d swivelPickPose = new Pose2d(new Vector2d(25.55, -33.71), Math.toRadians(swivelAngle));

            Action armPick = telemetryPacket -> {
                hand.release(600);
                return false;
            };
            Action armDrop = telemetryPacket -> {
                hand.release(600);
                return false;
            };

            Action snagIt1 = new ParallelAction(
                    new CompleteAction(legs.moveToAction(AUTO_POWER, swivelPickPose, false), legs)
            );

            int dropArmPosition = 2185;
            Pose2d swivelDropPose = new Pose2d(new Vector2d(25.55, -33.71), Math.toRadians(-156));

            Action dropIt = new SequentialAction(
                    new CompleteAction(legs.moveToAction(AUTO_POWER, swivelDropPose, true), legs),
                    new CompleteAction(release, hand)
            );
            Actions.runBlocking(dropIt);

        }
        /*
        Action toSwivel = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, swivelPose, false), legs),
        );
        Actions.runBlocking(toSwivel);


        Action avoidAndHover = new ParallelAction(

                new CompleteAction(armIn, arm),
                new CompleteAction(hoverShoulder, shoulder)
        );

        Action avoidAndHover = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, avoidSub, true), legs),
                new CompleteAction(armIn, arm),
                new CompleteAction(hoverShoulder, shoulder)
        );


        Pose2d dropAndPickup2 = new Pose2d(new Vector2d(8.0 + X_OFFSET, -32.5 + Y_OFFSET), Math.toRadians(-135));
        Pose2d dropAndPickupEnd = new Pose2d(new Vector2d(8.3 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));
        Pose2d secondHang = new Pose2d(new Vector2d(31 + X_OFFSET, 4.5+ Y_OFFSET), Math.toRadians(0));
        Pose2d avoidSub = new Pose2d(new Vector2d(22 + X_OFFSET, -24.2 + Y_OFFSET), Math.toRadians(0));
        Pose2d behind1 = new Pose2d(new Vector2d(46 + X_OFFSET, -39 + Y_OFFSET), Math.toRadians(180));
        Pose2d push1 = new Pose2d(new Vector2d(8 + X_OFFSET, -35 + Y_OFFSET), Math.toRadians(180));
        Pose2d behind2 = new Pose2d(new Vector2d(46 + X_OFFSET, -46 + Y_OFFSET), Math.toRadians(180));
        Pose2d push2 = new Pose2d(new Vector2d(10 + X_OFFSET, -50 + Y_OFFSET), Math.toRadians(180));
        Pose2d push2Backup = new Pose2d(new Vector2d(18 + X_OFFSET, -50 + Y_OFFSET), Math.toRadians(180));
        Pose2d safeSpot = new Pose2d(new Vector2d(18 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));
        Pose2d thirdHang = new Pose2d(new Vector2d(32 + X_OFFSET, 9 + Y_OFFSET), Math.toRadians(0));
        Pose2d repeatSecondHangPose = new Pose2d(new Vector2d(32 + X_OFFSET, 4.5+ Y_OFFSET), Math.toRadians(0));
        Pose2d park = new Pose2d(new Vector2d(8.3 + X_OFFSET, -32.2 + Y_OFFSET), Math.toRadians(-135));

        Pose2d searchPose = new Pose2d(new Vector2d(22.0 + X_OFFSET, Y_OFFSET), 0);

        // Common actions for the right side
        Action hoverShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, Shoulder.Mode.SEARCH.armInPos());
            return false;
        };

        Action armIn = telemetryPacket -> {
            arm.setPosition(AUTO_POWER, -15);
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
                    new CompleteAction(legs.moveToAction(AUTO_POWER, dropAndPickup2, -1), legs));
            Actions.runBlocking(gotoPickup);

            // Extra floor alignment

            Action adjust = eye.safeFloor();
            if(adjust != null)
                Actions.runBlocking(adjust);


            //eye.debugTelemetry(telemetry);
            //telemetry.update();

            Action grabAction = new SequentialAction(
                    ground,
                    new CompleteAction(grab, hand)
            );
            Actions.runBlocking(grabAction);

            Action gotoSecondHang = new ParallelAction(
                    new CompleteAction(liftShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, secondHang, 1), legs)
            );
            Actions.runBlocking(gotoSecondHang);

            // Extra bar alignment
            Action action = eye.safeHang();
            if(action != null)
                Actions.runBlocking(action);

            Action retractReleaseBackup = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(release, hand),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, true), legs)
            );


            Action dropAndRelease = new SequentialAction(
                    new CompleteAction(drop, shoulder),
                    retractReleaseBackup
            );
            Actions.runBlocking(dropAndRelease);
        }

        Action avoidAndHover = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, avoidSub, true), legs),
                new CompleteAction(armIn, arm),
                new CompleteAction(hoverShoulder, shoulder)
        );

        Action driveAction = new SequentialAction(
                avoidAndHover,
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, push1, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind2, true), legs),
                new CompleteAction(legs.moveToAction(0.8, push2, -1, true), legs),
                new CompleteAction(legs.moveToAction(0.8, push2Backup, -1, true), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, true), legs),
                new CompleteAction(legs.moveToAction(APPROACH_POWER, dropAndPickupEnd), legs)
        );
        Actions.runBlocking(driveAction);

        // Extra floor alignment
        Action adjust2 = eye.safeFloor();
        if(adjust2 != null)
            Actions.runBlocking(adjust2);
        //eye.debugTelemetry(telemetry);
        //telemetry.update();

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

        // Extra bar alignment
        Action action = eye.safeHang();
        if(action != null)
            Actions.runBlocking(action);

        Action retractReleaseBackup2 = new ParallelAction(
                new CompleteAction(armIn, arm),
                new CompleteAction(release, hand),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, true), legs)
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
                    new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, -1), legs),
                    new CompleteAction(legs.moveToAction(APPROACH_POWER, dropAndPickupEnd, -1), legs));
            Actions.runBlocking(finalPickup);

            // Extra floor alignment
            Action adjust3 = eye.safeFloor();
            if(adjust3 != null)
                Actions.runBlocking(adjust3);
            //eye.debugTelemetry(telemetry);
            //telemetry.update();

            Action grab3Action = new SequentialAction(
                    ground,
                    new CompleteAction(grab, hand)
            );
            Actions.runBlocking(grab3Action);

            Action repeatSecondHang = new ParallelAction(
                    new CompleteAction(liftShoulderAction, shoulder),
                    new CompleteAction(extendArmAction, arm),
                    new CompleteAction(legs.moveToAction(AUTO_POWER, repeatSecondHangPose, 1), legs)
            );
            Actions.runBlocking(repeatSecondHang);

            // Extra bar alignment
            Action action2 = eye.safeHang();
            if(action2 != null)
                Actions.runBlocking(action2);

            Action retractReleaseBackup3 = new ParallelAction(
                    new CompleteAction(armIn, arm),
                    new CompleteAction(release, hand),
                    new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, searchPose, true), legs)
            );

            Action dropAndRelease3 = new SequentialAction(
                    new CompleteAction(drop, shoulder),
                    retractReleaseBackup3
            );
            Actions.runBlocking(dropAndRelease3);
        }

        Actions.runBlocking(new CompleteAction(legs.moveToAction(AUTO_POWER, safeSpot, -1, false), legs));

        Action resetAction = new ParallelAction(
                new CompleteAction(resetArm, arm),
                new CompleteAction(hoverShoulder, shoulder),
                new CompleteAction(legs.moveToAction(APPROACH_POWER, dropAndPickupEnd, -1), legs));
        Actions.runBlocking(resetAction);

        // Extra floor alignment
        Action adjust4 = eye.safeFloor();
        if(adjust4 != null)
            Actions.runBlocking(adjust4);
        //eye.debugTelemetry(telemetry);
        //telemetry.update();

        Action extraResetAction = new ParallelAction(
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(grab, hand));
        Actions.runBlocking(extraResetAction);

         */
    }
}