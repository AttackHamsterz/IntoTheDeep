package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Speedy Hamster", group="Linear Opmode")
public class HamsterOpMode_Linear extends LinearOpMode {

    // Declare OpMode Members (Members of the house)
    //private DcMotor leftFoot = null;
    //private DcMotor rightFoot = null;
    private Servo tail = null;
    //private Servo linear = null;

    @Override
    public void runOpMode() {

        // To turn on the hardware.
        // Note the strings used at the end of code are parameters
        // to 'get' means to connect the names assigned during the robot configuaration
        // step (using the FTC Robot Controller app on the phones)
        //leftFoot = hardwareMap.get(DcMotor.class, "leftFoot");
        //rightFoot = hardwareMap.get(DcMotor.class, "rightFoot");
        tail = hardwareMap.get(Servo.class, "intake");
        //linear = hardwareMap.get(Servo.class, "linear");


        // To drive forward sometimes the motors are flipped but pushing the left stick forward
        // must make the robot go forward. Adjust as needed to make this happen.
        //leftFoot.setDirection(DcMotor.Direction.FORWARD);
        //rightFoot.setDirection(DcMotor.Direction.FORWARD);
        tail.setDirection(Servo.Direction.FORWARD);
        //linear.setDirection(Servo.Direction.FORWARD);

        //HamsterMotion hamsterMotion = new HamsterMotion(leftFoot, rightFoot, tail, gamepad1);

        // Wait for driver to press play
        waitForStart();
/*
        while (!isInterrupted()) {

            if (gamepad1.x) {
                tail.setpower(1.0);
            }
            else{
                tail.setpower(0.0);
            }
            */
            // Lunch Motion
        //hamsterMotion.start();

        // run until driver pushes stop
        while (opModeIsActive()){

            // Show the wheel power and servo angel
            telemetry.addData("Servo Angle", "(%.2f)", tail.getPosition());
            telemetry.addData("X Pressed", gamepad1.x);
            telemetry.addData("Y Pressed", gamepad1.y);
            telemetry.update();

        }
    }
}