package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Four Corners", group = "Robot")
public final class FourCorners extends StandardSetupOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        Pose2d pose0 = new Pose2d( 17, 40, Math.toRadians(90));
        Pose2d pose1 = new Pose2d( 17+94.5, 40, Math.toRadians(0));
        Pose2d pose2 = new Pose2d( 17+94.5, 40-94.5, Math.toRadians(-90));
        Pose2d pose3 = new Pose2d( 17, 40-94.5, Math.toRadians(180));

        Action testAction = new SequentialAction(
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose0), legs, 10000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose1), legs, 10000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose2), legs, 10000),
                new CompleteAction(legs.moveToAction(AUTO_MOVE_POWER, pose3), legs, 10000)
                );
        Actions.runBlocking(testAction);
    }
}
