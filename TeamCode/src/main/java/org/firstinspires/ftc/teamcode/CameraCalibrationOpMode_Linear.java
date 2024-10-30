package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;

@TeleOp(name = "Test: Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends LinearOpMode {

    HuskyLens huskyLens;
    ColorCamera camera;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    private DcMotor shoulderMotor = null;
    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;


    private HuskyLens.Block closestBlock;
    private int closestBlockX;
    private int closestBlockY;


    @Override
    public void runOpMode() throws InterruptedException {

        huskyLens = hardwareMap.get(HuskyLens.class, "camera");
        camera = new ColorCamera(huskyLens, "red");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0

        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotorLeft.setTargetPosition(0);
        armMotorRight.setTargetPosition(0);
        shoulderMotor.setTargetPosition(0);


        waitForStart();

        Arm arm = new Arm(armMotorLeft, armMotorRight, gamepad2, null);
        Shoulder shoulder = new Shoulder(shoulderMotor, arm, gamepad2);
        arm.setShoulder(shoulder);

        shoulder.start();
        arm.start();

        while (opModeIsActive()) {

            shoulder.interrupt();
            arm.interrupt();
            try {
                shoulder.join();
                arm.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shoulder.setPosition(1, Shoulder.SEARCH_HEIGHT);

            telemetry.addData("running", true);

            closestBlock = camera.getClosestBlock();
            if (closestBlock != null) {
                closestBlockX = closestBlock.x;
                closestBlockY = closestBlock.y;

                int deltaX = ColorCamera.CENTER_X - closestBlockX;
                int deltaY = ColorCamera.CENTER_Y - closestBlockY;

                double powerX = (double) (deltaX / (double) (ColorCamera.CENTER_X / 2));
                double powerY = (double) (deltaY / (double) (ColorCamera.CENTER_Y / 2));

                telemetry.addData("powerX" , powerX);
                telemetry.addData("powerY", powerY);

                //frontLeftDrive.setPower(powerY);
                //frontRightDrive.setPower(powerY);
                //rearLeftDrive.setPower(powerY);
                //rearRightDrive.setPower(powerY);
            } else {
                //frontLeftDrive.setPower(0);
                //frontRightDrive.setPower(0);
                //rearLeftDrive.setPower(0);
                //rearRightDrive.setPower(0);
            }

            telemetry.addData("Closest Block", closestBlock);

            ArrayList<HuskyLens.Block> blocks = camera.getAllBlocks();
            for (int i = 0; i < blocks.size(); i++) {
                telemetry.addData("Blocks: ", blocks.get(i));
            }
            telemetry.update();

        }
    }
}
