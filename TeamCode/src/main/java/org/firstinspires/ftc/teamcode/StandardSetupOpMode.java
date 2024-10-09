package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
public class StandardSetupOpMode {
    // Declare Robot Setup Members.
    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor rearLeftDrive = null;
    protected DcMotor rearRightDrive = null;

    protected Motion motion = null;
    protected Arm arm = null;
    protected Shoulder shoulder = null;

    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;
    private DcMotor shoulderMotor = null;

   // @Override
    public void runOpMode() throws InterruptedException {
        //Call camera setup
        //super.runOpMode();

        //Initialize the hardware variables
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft"); //ch1 expansion hub Motor
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight"); //ch2 expansion hub Motor
        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor"); //ch0 Motor

        //setting the motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        armMotorRight.setDirection(DcMotor.Direction.REVERSE);
        shoulderMotor.setDirection(DcMotor.Direction.REVERSE);

        //setting type of run mode --> different ways to have ur motor run
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Build the classes and give in an object
        motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, null);
        arm = new Arm(armMotorLeft, armMotorRight, null);
        shoulder = new Shoulder(shoulderMotor, gamepad2);


    }



}




