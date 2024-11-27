package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.CompleteAction;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.StandardSetupOpMode;

public final class SplineTest extends StandardSetupOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        Pose2d pose0 = new Pose2d( 0, 0, Math.toRadians(0));
        Pose2d pose1 = new Pose2d( 14, 0, Math.toRadians(90));
        Pose2d pose2 = new Pose2d( 20, 14, Math.toRadians(0));
        Pose2d pose3 = new Pose2d( 20, -14, Math.toRadians(270));
        Pose2d pose4 = new Pose2d( 14, 0, Math.toRadians(180));
        Pose2d pose5 = new Pose2d( 20, 0, Math.toRadians(-45));
        Pose2d pose6 = new Pose2d( 10, 0, Math.toRadians(45));

        Action testAction = new SequentialAction(
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose1), legs, 5000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose2), legs, 5000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose3), legs, 5000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose4), legs, 5000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose5), legs, 5000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose6), legs, 5000)
                );
        Actions.runBlocking(testAction);

        // Come back
        Actions.runBlocking(new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose0), legs));
    }
}
