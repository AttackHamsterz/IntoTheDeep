package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public class AutonomousBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d (0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // move forward towards the submersible
                            .splineTo(new Vector2d(20,0),0)
                            // raise arm to place specimen
                            // place specimen on the bar
                            .waitSeconds(2.0)
                            // turn towards yellow samples
                            .turnTo(Math.PI / 2)
                            // extend the arm to grab one sample
                            .waitSeconds(2.0)
                            //turn to face basket
                            .turnTo(Math.toRadians(-(Math.PI / 2)))
                            // move towards basket
                            .splineTo(new Vector2d(20, 40), -(Math.PI / 2))
                            // raise arm
                            // place sample in basket
                            .build()
            );

        }
    }

}
