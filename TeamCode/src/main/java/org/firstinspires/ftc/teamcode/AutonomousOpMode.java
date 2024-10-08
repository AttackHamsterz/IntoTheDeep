package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

//extends standard setup op mode
public class AutonomousOpMode extends StandardSetupOpMode{

    public enum COLOR {
        RED,
        BLUE
    }

    public enum START_POS {
        FAR,
        NEAR
    }

   protected MecanumDrive drive;

   @Override
   public void runOpMode() throws InterruptedException {
       //Parent opmode call
       super.runOpMode();
   }

   protected void setup(HardwareMap hardwareMap, START_POS startPos, COLOR color) {
       //variable setup
       }

}
