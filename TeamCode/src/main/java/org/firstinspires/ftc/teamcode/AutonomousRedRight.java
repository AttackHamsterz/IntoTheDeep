package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Red Right", group = "Robot")
public class AutonomousRedRight extends AutonomousRightFast {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(AutonomousOpMode.COLOR.RED, AutonomousOpMode.SIDE.RIGHT, true, false);
        super.runOpMode();
    }
}
