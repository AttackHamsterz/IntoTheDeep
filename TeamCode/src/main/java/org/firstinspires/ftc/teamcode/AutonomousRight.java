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

        Pose2d dropAndPickup = new Pose2d(new Vector2d(8.3, -32.2), Math.toRadians(-135));
        Pose2d secondHang = new Pose2d(new Vector2d(22.5, 2), Math.toRadians(0));
        Pose2d avoidSub = new Pose2d(new Vector2d(25.5, -24.2), Math.toRadians(0));
        Pose2d behind1 = new Pose2d(new Vector2d(46.5, -39), Math.toRadians(180));
        Pose2d push1 = new Pose2d(new Vector2d(8, -35), Math.toRadians(180));
        Pose2d behind2 = new Pose2d(new Vector2d(46.5, -47), Math.toRadians(180));
        Pose2d push2 = new Pose2d(new Vector2d(8, -47), Math.toRadians(180));
        Pose2d thirdHang = new Pose2d(new Vector2d(23, 4), Math.toRadians(0));
        Pose2d park = new Pose2d(new Vector2d(8.3, -32.2), Math.toRadians(-135));

        // No override mode for shoulder
        shoulder.setMode(Shoulder.Mode.NONE);

        Action search = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, Shoulder.Mode.SEARCH.armInPos());
            return false;
        };

        Action armIn = telemetryPacket -> {
            arm.gotoMin(AUTO_POWER);
            return false;
        };

        Action action1 = new ParallelAction(
                new CompleteAction(search, shoulder),
                new CompleteAction(armIn, arm),
                new CompleteAction(legs.moveToAction(AUTO_POWER, dropAndPickup), legs));
        Actions.runBlocking(action1);

        Action grab = telemetryPacket -> {
            hand.grab(800);
            return false;
        };

        Action ground = telemetryPacket -> {
            shoulder.setPosition(0.8, Shoulder.Mode.GROUND.armInPos());
            return false;
        };

        Action action2 = new SequentialAction(
                ground,
                new CompleteAction(grab, hand)
        );
        Actions.runBlocking(action2);

        Action liftShoulderAction = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, dropShoulderPositionTop);
            return false;
        };

        Action extendArmAction = telemetryPacket -> {
            arm.setPosition(AUTO_POWER,dropArmPosition);
            return false;
        };

        Actions.runBlocking(new CompleteAction(search, shoulder));

        Action spinHand = telemetryPacket -> {
            hand.grab(500);
            return false;
        };

        Action action3 = new ParallelAction(
                new CompleteAction(liftShoulderAction, shoulder),
                new CompleteAction(extendArmAction, arm),
                new CompleteAction(legs.moveToAction(AUTO_POWER, secondHang, 1), legs)
                //new CompleteAction(spinHand, hand)
        );
        Actions.runBlocking(action3);

        Action drop = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 1450);
            return false;
        };

        Action release = telemetryPacket -> {
            hand.release(500);
            return false;
        };

        Action action4 = new SequentialAction(
                new CompleteAction(drop, shoulder),
                new CompleteAction(release, hand),
                new CompleteAction(armIn, arm)
        );
        Actions.runBlocking(action4);

        Action driveAction = new SequentialAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, avoidSub), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1, -1), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, push1, -1), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind2), legs),
                new CompleteAction(legs.moveToAction(0.8, push2, -1), legs)
        );
        Actions.runBlocking(driveAction);

        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };
        Action resetAction = new ParallelAction(
                new CompleteAction(legs.moveToAction(AUTO_POWER, park, 1), legs),
                new CompleteAction(resetShoulder, shoulder),
                new CompleteAction(resetArm, arm));
        Actions.runBlocking(resetAction);
    }
}