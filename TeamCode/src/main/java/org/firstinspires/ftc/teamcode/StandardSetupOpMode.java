package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;
    private DcMotor shoulderMotor = null;
    private Servo wristServo = null;
    private CRServo leftHandServo = null;
    private CRServo rightHandServo = null;


   // @Override
    public void runOpMode() throws InterruptedException {
        //Call camera setup
        //super.runOpMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note
        // that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 expansion hub Motor
        wristServo = hardwareMap.get(Servo.class, "wristServo"); //ch0 expansion hub Servo
        leftHandServo = hardwareMap.get(CRServo.class, "leftHandServo"); //ch1 expansion hub Servo
        rightHandServo = hardwareMap.get(CRServo.class, "rightHandServo"); //ch2 expansion hub Servo

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        Motion motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, gamepad1);
        Arm arm = new Arm(armMotorLeft, armMotorRight, gamepad2, null);
        Shoulder shoulder = new Shoulder(shoulderMotor, arm, gamepad2);
        arm.setShoulder(shoulder);
        Hand hand = new Hand(leftHandServo, rightHandServo, wristServo, gamepad2);

        motion.start();
        shoulder.start();
        arm.start();
        hand.start();





    }



}




