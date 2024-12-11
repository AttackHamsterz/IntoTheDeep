package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Blue Right - Partner has Specimen", group = "Robot")
public class AutonomousBlueRightPartnerHasSpecimen extends AutonomousRight
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        setup(COLOR.BLUE, SIDE.RIGHT, true, false);
        partnerHasSpecimen();
        super.runOpMode();
    }
}
