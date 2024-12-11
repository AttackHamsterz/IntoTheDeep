package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Red Right Partner Has Specimen", group = "Robot")
public class AutonomousRedRightPartnerHasSpecimen extends AutonomousRight {
    @Override
    public void runOpMode() throws InterruptedException {
        setup(COLOR.RED, SIDE.RIGHT, true, false);
        partnerHasSpecimen();
        super.runOpMode();
    }
}
