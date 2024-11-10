package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Blue Left", group = "Robot")
public class AutonomousBlueLeft extends AutonomousLeft
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(COLOR.BLUE, SIDE.LEFT, true);
        super.runOpMode();
    }
}
