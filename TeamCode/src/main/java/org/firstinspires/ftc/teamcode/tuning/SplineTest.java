package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.Shoulder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shoulder shoulder = new Shoulder(hardwareMap, null, gamepad2);
        shoulder.setPosition(0.9, 200);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, gamepad1);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // move forward towards the submersible
                            //.splineTo(new Vector2d(7,0),0)
                            // raise arm to place specimen
                            // place specimen on the bar
                            .waitSeconds(1.0)
                            // turn towards yellow samples
                            //move towards samples
                            //.strafeToLinearHeading(new Vector2d(7+11*Math.sqrt(2) , -11*Math.sqrt(2)), -(Math.PI / 4))
                            // extend the arm to grab one sample
                            //.waitSeconds(2.0)
                            //turn to face basket
                            //.turn(Math.PI)
                            //.waitSeconds(2.0)
                            // move towards basket
                            .lineToX(48)
                            //.strafeTo(new Vector2d(0, 24))
                            .waitSeconds(1.0)
                            .turn(Math.toRadians(-90))
                            .waitSeconds(1.0)
                            .lineToY(-40)
                            //.turn(-Math.PI)
                            .build()
            );
        } else {
            throw new RuntimeException();
        }
    }
}
