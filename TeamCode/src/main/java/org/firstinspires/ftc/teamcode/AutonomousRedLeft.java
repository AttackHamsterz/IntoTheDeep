package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Red Left", group = "Robot")
public class AutonomousRedLeft extends AutonomousLeftFast {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, SIDE.LEFT, true, true);
        searchSubmersible();
        super.runOpMode();
    }
}
