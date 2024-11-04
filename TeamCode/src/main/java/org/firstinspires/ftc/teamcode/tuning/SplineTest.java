package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, gamepad1);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
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
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(20, 20), Math.PI / 2)
                            .splineTo(new Vector2d(0, 0), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
