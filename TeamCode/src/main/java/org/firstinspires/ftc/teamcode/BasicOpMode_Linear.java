/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor frontLeftDrive = null;
    //private DcMotor frontRightDrive = null;
    //private DcMotor rearLeftDrive = null;
    //private DcMotor rearRightDrive = null;
    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;
    private DcMotor shoulderMotor = null;
    private Servo wristServo = null;
    private CRServo leftHandServo = null;
    private CRServo rightHandServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note
        // that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        //frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        //rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        //rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor
        wristServo = hardwareMap.get(Servo.class, "wristServo"); //ch0 expansion hub Servo
        leftHandServo = hardwareMap.get(CRServo.class, "leftHandServo"); //ch1 expansion hub Servo
        rightHandServo = hardwareMap.get(CRServo.class, "rightHandServo"); //ch2 expansion hub Servo

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        //frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        wristServo.setDirection(Servo.Direction.FORWARD);
        leftHandServo.setDirection(CRServo.Direction.FORWARD);
        rightHandServo.setDirection(CRServo.Direction.FORWARD);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotorLeft.setTargetPosition(0);
        armMotorRight.setTargetPosition(0);
        shoulderMotor.setTargetPosition(0);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        // Launch Threads
        //Motion motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, gamepad1);
        MecanumDrive motion = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), gamepad1);
        Arm arm = new Arm(armMotorLeft, armMotorRight, gamepad2, null);
        Shoulder shoulder = new Shoulder(shoulderMotor, arm, gamepad2);
        arm.setShoulder(shoulder);
        Hand hand = new Hand(leftHandServo, rightHandServo, wristServo, gamepad2);

        motion.start();
        shoulder.start();
        arm.start();
        hand.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Stick", "x (%.2f), y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Right Stick", "x (%.2f), y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Left Stick", "x (%.2f), y (%.2f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("Right Stick", "x (%.2f), y (%.2f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            //telemetry.addData("Front Left Motor", "(%7d)", frontLeftDrive.getCurrentPosition());
            //telemetry.addData("Front Right Motor", "(%7d)", frontRightDrive.getCurrentPosition());
            //telemetry.addData("Rear Left Motor", "(%7d)", rearLeftDrive.getCurrentPosition());
            //telemetry.addData("Rear Right Motor", "(%7d)", rearRightDrive.getCurrentPosition());
            telemetry.addData("Shoulder Motor", "(%7d)", shoulderMotor.getCurrentPosition());
            telemetry.addData("Arm Counts Left", armMotorLeft.getCurrentPosition());
            telemetry.addData("Arm Counts Right", armMotorRight.getCurrentPosition());
            telemetry.addData("Arm Ratio", arm.getArmRatio());
            telemetry.addData("Shoulder Ratio", shoulder.getShoulderRatio());
            telemetry.addData("Left Hand Power", leftHandServo.getPower());
            telemetry.addData("Right Hand Power", rightHandServo.getPower());
            telemetry.update();
        }
        motion.interrupt();
        shoulder.interrupt();
        arm.interrupt();
        hand.interrupt();
        try {
            motion.join();
            shoulder.join();
            arm.join();
            hand.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}


