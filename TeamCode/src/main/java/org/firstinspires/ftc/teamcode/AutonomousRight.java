package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

public class AutonomousRight extends AutonomousOpMode{
    protected double partnerWaitForSeconds = 0.0;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();
        super.setup(hardwareMap, START_POS.RIGHT);

        Actions.runBlocking(
                drive.actionBuilder(super.startPose)
                        // move forward towards the submersible
                        .splineTo(new Vector2d(7,0),0)
                        // raise arm to place specimen
                        // place specimen on the bar
                        .waitSeconds(2.0)
                        // turn towards yellow samples
                        //move towards samples
                        .strafeToLinearHeading(new Vector2d(7+11*Math.sqrt(2) , -11*Math.sqrt(2)), -(Math.PI / 4))
                        // extend the arm to grab one sample
                        .waitSeconds(2.0)
                        //turn to face basket
                        .turn(Math.PI)
                        .waitSeconds(2.0)
                        // move towards basket
                        //.lineToY(40)
                        // raise arm
                        // place sample in basket
                        .build()
        );
    }
}