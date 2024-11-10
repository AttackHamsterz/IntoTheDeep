package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Blue Right", group = "Robot")
public class AutonomousBlueRight extends AutonomousRight
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(AutonomousOpMode.COLOR.BLUE, AutonomousOpMode.SIDE.RIGHT, true);
        super.runOpMode();
    }
}
