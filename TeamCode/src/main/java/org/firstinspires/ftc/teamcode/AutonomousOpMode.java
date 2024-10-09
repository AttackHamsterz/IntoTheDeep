package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

//extends standard setup op mode
public class AutonomousOpMode extends StandardSetupOpMode{

    public enum COLOR {
        RED,
        BLUE
    }

    public enum START_POS {
        LEFT,
        RIGHT
    }

   protected MecanumDrive drive;

    //poses
   protected Pose2d startPose = new Pose2d(0,0,0);
   protected START_POS startPos;

   @Override
   public void runOpMode() throws InterruptedException {
       //Parent opmode call
       super.runOpMode();
   }

   //Add color as a paramater later
   protected void setup(HardwareMap hardwareMap, START_POS startPos) {
       //variable setup
       drive = new MecanumDrive(hardwareMap, startPose);
       this.startPos = startPos;

       //Class Setup
       // super.arm.setShoulder(shoulder);
       //super.arm.ignoreGamepad();
       super.shoulder.ignoreGamepad();
       super.shoulder.start();
   }

}
