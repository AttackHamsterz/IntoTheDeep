package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        Pose2d dropAndPickup = new Pose2d(new Vector2d(12, -40), Math.toRadians(-135));
        Pose2d secondHang = new Pose2d(new Vector2d(23, 2), Math.toRadians(0));
        Pose2d avoidSub = new Pose2d(new Vector2d(23, -40), Math.toRadians(0));
        Pose2d behind1 = new Pose2d(new Vector2d(40, -50), Math.toRadians(180));
        Pose2d push1 = new Pose2d(new Vector2d(8, -50), Math.toRadians(180));
        Pose2d behind2 = new Pose2d(new Vector2d(40, -50), Math.toRadians(180));
        Pose2d push2 = new Pose2d(new Vector2d(8, -60), Math.toRadians(180));
        Pose2d thirdHang = new Pose2d(new Vector2d(23, 4), Math.toRadians(0));
        Pose2d park = new Pose2d(new Vector2d(12, -40), Math.toRadians(-135));

        Action armAndShoulderPickup = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.SEARCH);
            arm.gotoMin(AUTO_POWER);
            return false;
        };

        Action pickup = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.GROUND);
            hand.grab(800);
            return false;
        };

        Action readyToHang = telemetryPacket -> {
            shoulder.setMode(Shoulder.Mode.HIGH_BAR);
            return false;
        };

        Action drop = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 1450);
            return false;
        };

        Action resetShoulder = telemetryPacket -> {
            shoulder.setPosition(AUTO_POWER, 0);
            return false;
        };

        Action resetArm = telemetryPacket -> {
            hand.hangSample();
            arm.setPosition(AUTO_POWER, 0);
            return false;
        };

        Action reset = new ParallelAction(
                new CompleteAction(resetArm, arm),
                new CompleteAction(resetShoulder, shoulder)
        );

        Action rightAction = new SequentialAction(
                armAndShoulderPickup,
                new CompleteAction(legs.moveToAction(AUTO_POWER, dropAndPickup), legs),
                new CompleteAction(pickup, hand),
                readyToHang,
                new CompleteAction(legs.moveToAction(AUTO_POWER, secondHang), legs),
                new CompleteAction(drop, shoulder),
                new CompleteAction(legs.moveToAction(AUTO_POWER, avoidSub), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind1), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, push1), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, behind2), legs),
                new CompleteAction(legs.moveToAction(AUTO_POWER, push2), legs),
                new CompleteAction(pickup, hand),
                readyToHang,
                new CompleteAction(legs.moveToAction(AUTO_POWER, thirdHang), legs),
                new CompleteAction(drop, shoulder),
                new CompleteAction(legs.moveToAction(AUTO_POWER, park), legs),
                reset
        );
        Actions.runBlocking(rightAction);
    }
}