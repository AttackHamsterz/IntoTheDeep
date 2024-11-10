package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Red Left", group = "Robot")
public class AutonomousRedLeft extends AutonomousLeft {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(AutonomousOpMode.COLOR.RED, AutonomousOpMode.SIDE.LEFT, true);
        super.runOpMode();
    }
}
