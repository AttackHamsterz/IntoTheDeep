package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Motion extends Thread {
    public static final int LOW_POSITION = 3941;
    public static final int MIDDLE_POSITION = 6647;
    public static final int HIGH_POSITION = 9700;

    public static final double AUTO_SPEED = 0.6;

    public static int TRANSLATE_FB = 1060;
    public static int TRANSLATE_LR = 1200;

    public static int ROTATE_360 = 3800;

    private double PF = 1.0;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor rearLeftDrive;
    private final DcMotor rearRightDrive;
    private final Gamepad gamepad;

    protected MecanumDrive legs;

    public enum Direction {
        FORWARD,
        RIGHT,
        BACKWARD,
        LEFT
    }

    public Motion(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor rearLeftDrive, DcMotor rearRightDrive, Gamepad gamepad, MecanumDrive legs) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.rearLeftDrive = rearLeftDrive;
        this.rearRightDrive = rearRightDrive;
        this.gamepad = gamepad;
        this.legs = legs;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Debug

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double rearLeftPower;
            double rearRightPower;

            // POV Mode uses left stick to go forward and strafe, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = -gamepad.right_stick_x;

            frontLeftPower = Range.clip(drive + turn - strafe, -1.0, 1.0) * PF;
            rearLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0) * PF;
            frontRightPower = Range.clip(drive - turn + strafe, -1.0, 1.0) * PF;
            rearRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0) * PF;

            if (gamepad.right_trigger > 0) {
                PF = 0.25;
            } else if (gamepad.left_trigger > 0) {
                PF = 1.5;
            } else {
                PF = 1.0;
            }

            if (gamepad.right_bumper) {
                rotate(90);
            } else if (gamepad.left_bumper) {
                rotate(-90);
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            rearLeftDrive.setPower(rearLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearRightDrive.setPower(rearRightPower);
        }
    }

    public void rotate(double degrees) {
        Action rotate = legs.actionBuilder(legs.pose)
                .turn(Math.toRadians(degrees))
                .build();
        Actions.runBlocking(rotate);
    }
}


